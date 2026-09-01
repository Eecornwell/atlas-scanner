import Foundation
import Compression

/// Uploads a session zip to a host running enhance_server.py,
/// streams log output back, and downloads the enhanced results.
@MainActor
final class HostUploader: NSObject, ObservableObject {
    @Published var progress = ExportProgress()
    @Published var logLines: [String] = []

    private var uploadTask: URLSessionUploadTask?
    private var downloadTask: URLSessionDownloadTask?

    // MARK: - Public

    func upload(sessionDirectory: URL, hostURL: String) async {
        guard let baseURL = URL(string: hostURL) else {
            progress.error = "Invalid host URL: \(hostURL)"
            return
        }

        progress = ExportProgress(stage: "Zipping session…", total: 3)

        guard let zipURL = await zipSession(sessionDirectory) else {
            progress.error = "Failed to create session zip"
            return
        }
        progress.current = 1
        progress.stage = "Uploading to host…"

        let uploadEndpoint = baseURL.appendingPathComponent("upload")
        guard let jobID = await postZip(zipURL: zipURL, to: uploadEndpoint) else {
            progress.error = "Upload failed — is enhance_server.py running on the host?"
            try? FileManager.default.removeItem(at: zipURL)
            return
        }
        progress.current = 2
        progress.stage = "Processing on host…"

        let statusEndpoint = baseURL.appendingPathComponent("status/\(jobID)")
        let success = await pollStatus(endpoint: statusEndpoint)
        guard success else {
            progress.error = "Host processing failed — check server logs"
            return
        }

        progress.stage = "Downloading results…"
        let downloadEndpoint = baseURL.appendingPathComponent("download/\(jobID)")
        await downloadResults(from: downloadEndpoint, into: sessionDirectory)

        progress = ExportProgress(stage: "Complete", current: 3, total: 3, isComplete: true)
        try? FileManager.default.removeItem(at: zipURL)
    }

    func cancel() {
        uploadTask?.cancel()
        downloadTask?.cancel()
        progress.error = "Cancelled"
    }

    // MARK: - Zip

    private func zipSession(_ sessionDirectory: URL) async -> URL? {
        await Task.detached(priority: .userInitiated) {
            let zipURL = FileManager.default.temporaryDirectory
                .appendingPathComponent("\(sessionDirectory.lastPathComponent).zip")
            try? FileManager.default.removeItem(at: zipURL)
            return ZipWriter.write(sourceDir: sessionDirectory, to: zipURL) ? zipURL : nil
        }.value
    }

    // MARK: - Upload

    private func postZip(zipURL: URL, to endpoint: URL) async -> String? {
        guard let zipData = try? Data(contentsOf: zipURL) else { return nil }
        var request = URLRequest(url: endpoint)
        request.httpMethod = "POST"
        request.setValue("application/zip", forHTTPHeaderField: "Content-Type")
        request.setValue(zipURL.lastPathComponent, forHTTPHeaderField: "X-Session-Name")
        do {
            let (data, response) = try await URLSession.shared.upload(for: request, from: zipData)
            guard (response as? HTTPURLResponse)?.statusCode == 200 else { return nil }
            let json = try? JSONSerialization.jsonObject(with: data) as? [String: String]
            return json?["job_id"]
        } catch {
            return nil
        }
    }

    // MARK: - Status polling

    private func pollStatus(endpoint: URL) async -> Bool {
        for _ in 0..<360 {
            try? await Task.sleep(nanoseconds: 5_000_000_000)
            guard let (data, _) = try? await URLSession.shared.data(from: endpoint),
                  let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any]
            else { continue }

            if let newLines = json["log_lines"] as? [String] {
                let existing = Set(logLines)
                for line in newLines where !existing.contains(line) {
                    await MainActor.run { logLines.append(line) }
                }
            }
            let status = json["status"] as? String ?? ""
            if status == "complete" { return true }
            if status == "error"    { return false }
        }
        return false
    }

    // MARK: - Download results

    private func downloadResults(from endpoint: URL, into sessionDirectory: URL) async {
        guard let (tmpURL, _) = try? await URLSession.shared.download(from: endpoint) else {
            progress.error = "Failed to download results"
            return
        }
        let destURL = sessionDirectory.appendingPathComponent("enhanced_host")
        try? FileManager.default.createDirectory(at: destURL, withIntermediateDirectories: true)
        ZipReader.extract(zipURL: tmpURL, to: destURL)
        try? FileManager.default.removeItem(at: tmpURL)
    }
}

// MARK: - Pure-Swift zip writer (Deflate via Compression framework)

/// Writes a ZIP archive from a directory tree.
/// Uses DEFLATE compression via Apple's Compression framework (built-in, iOS 13+).
/// Produces a standard ZIP file compatible with all unzip tools.
private enum ZipWriter {

    static func write(sourceDir: URL, to destURL: URL) -> Bool {
        var entries: [(localPath: URL, zipPath: String)] = []
        guard let enumerator = FileManager.default.enumerator(
            at: sourceDir,
            includingPropertiesForKeys: [.isRegularFileKey],
            options: [.skipsHiddenFiles]
        ) else { return false }

        let base = sourceDir.lastPathComponent
        for case let fileURL as URL in enumerator {
            guard (try? fileURL.resourceValues(forKeys: [.isRegularFileKey]))?.isRegularFile == true
            else { continue }
            let rel = fileURL.path.dropFirst(sourceDir.deletingLastPathComponent().path.count + 1)
            entries.append((fileURL, String(rel)))
        }

        var archive = Data()
        var centralDirectory = Data()
        var offsets: [UInt32] = []

        for entry in entries {
            guard let fileData = try? Data(contentsOf: entry.localPath) else { continue }
            let compressed = deflate(fileData)
            let useDeflate = compressed.count < fileData.count
            let payload = useDeflate ? compressed : fileData
            let method: UInt16 = useDeflate ? 8 : 0

            let nameData = entry.zipPath.data(using: .utf8) ?? Data()
            let crc = crc32(fileData)
            let offset = UInt32(archive.count)
            offsets.append(offset)

            // Local file header
            archive += localHeader(
                name: nameData, method: method,
                crc: crc, compSize: UInt32(payload.count),
                uncompSize: UInt32(fileData.count)
            )
            archive += payload

            // Central directory entry
            centralDirectory += centralEntry(
                name: nameData, method: method,
                crc: crc, compSize: UInt32(payload.count),
                uncompSize: UInt32(fileData.count),
                offset: offset
            )
        }

        let cdOffset = UInt32(archive.count)
        let cdSize   = UInt32(centralDirectory.count)
        archive += centralDirectory
        archive += endOfCentralDirectory(
            count: UInt16(entries.count), cdSize: cdSize, cdOffset: cdOffset
        )

        do {
            try archive.write(to: destURL)
            return true
        } catch {
            return false
        }
    }

    // MARK: - Deflate

    private static func deflate(_ input: Data) -> Data {
        // Raw deflate (no zlib header) — ZIP uses method 8 = raw deflate
        let bufSize = input.count + 1024
        var output = Data(count: bufSize)
        let written = input.withUnsafeBytes { src in
            output.withUnsafeMutableBytes { dst in
                compression_encode_buffer(
                    dst.baseAddress!.assumingMemoryBound(to: UInt8.self), bufSize,
                    src.baseAddress!.assumingMemoryBound(to: UInt8.self), input.count,
                    nil, COMPRESSION_ZLIB
                )
            }
        }
        // Strip the 2-byte zlib header and 4-byte adler32 trailer
        guard written > 6 else { return input }
        return output.subdata(in: 2..<(written - 4))
    }

    // MARK: - CRC-32

    private static func crc32(_ data: Data) -> UInt32 {
        var crc: UInt32 = 0xFFFFFFFF
        for byte in data {
            crc ^= UInt32(byte)
            for _ in 0..<8 {
                crc = (crc & 1) != 0 ? (crc >> 1) ^ 0xEDB88320 : crc >> 1
            }
        }
        return crc ^ 0xFFFFFFFF
    }

    // MARK: - ZIP structure builders

    private static func le16(_ v: UInt16) -> Data {
        var x = v.littleEndian; return Data(bytes: &x, count: 2)
    }
    private static func le32(_ v: UInt32) -> Data {
        var x = v.littleEndian; return Data(bytes: &x, count: 4)
    }

    private static func localHeader(
        name: Data, method: UInt16, crc: UInt32,
        compSize: UInt32, uncompSize: UInt32
    ) -> Data {
        var d = Data()
        d += le32(0x04034b50)   // signature
        d += le16(20)           // version needed
        d += le16(0)            // flags
        d += le16(method)
        d += le16(0); d += le16(0)  // mod time/date
        d += le32(crc)
        d += le32(compSize)
        d += le32(uncompSize)
        d += le16(UInt16(name.count))
        d += le16(0)            // extra length
        d += name
        return d
    }

    private static func centralEntry(
        name: Data, method: UInt16, crc: UInt32,
        compSize: UInt32, uncompSize: UInt32, offset: UInt32
    ) -> Data {
        var d = Data()
        d += le32(0x02014b50)   // signature
        d += le16(20); d += le16(20)  // version made/needed
        d += le16(0)            // flags
        d += le16(method)
        d += le16(0); d += le16(0)  // mod time/date
        d += le32(crc)
        d += le32(compSize)
        d += le32(uncompSize)
        d += le16(UInt16(name.count))
        d += le16(0); d += le16(0)  // extra, comment
        d += le16(0); d += le16(0)  // disk start, int attrs
        d += le32(0)            // ext attrs
        d += le32(offset)
        d += name
        return d
    }

    private static func endOfCentralDirectory(
        count: UInt16, cdSize: UInt32, cdOffset: UInt32
    ) -> Data {
        var d = Data()
        d += le32(0x06054b50)   // signature
        d += le16(0); d += le16(0)  // disk numbers
        d += le16(count); d += le16(count)
        d += le32(cdSize)
        d += le32(cdOffset)
        d += le16(0)            // comment length
        return d
    }
}

// MARK: - Pure-Swift zip reader (stored + deflate)

/// Extracts a ZIP archive to a destination directory.
/// Supports stored (method 0) and deflate (method 8) entries.
private enum ZipReader {

    static func extract(zipURL: URL, to destDir: URL) {
        guard let data = try? Data(contentsOf: zipURL) else { return }
        var offset = 0

        while offset + 30 < data.count {
            let sig = data.le32(at: offset)
            guard sig == 0x04034b50 else { break }  // local file header

            let method    = data.le16(at: offset + 8)
            let crc32     = data.le32(at: offset + 14)
            let compSize  = Int(data.le32(at: offset + 18))
            let uncompSize = Int(data.le32(at: offset + 22))
            let nameLen   = Int(data.le16(at: offset + 26))
            let extraLen  = Int(data.le16(at: offset + 28))

            let nameStart = offset + 30
            let nameEnd   = nameStart + nameLen
            guard nameEnd <= data.count else { break }
            let name = String(data: data[nameStart..<nameEnd], encoding: .utf8) ?? ""

            let payloadStart = nameEnd + extraLen
            let payloadEnd   = payloadStart + compSize
            guard payloadEnd <= data.count else { break }
            let payload = data[payloadStart..<payloadEnd]

            offset = payloadEnd

            // Skip directory entries
            if name.hasSuffix("/") { continue }

            let outURL = destDir.appendingPathComponent(name)
            try? FileManager.default.createDirectory(
                at: outURL.deletingLastPathComponent(),
                withIntermediateDirectories: true
            )

            switch method {
            case 0:  // stored
                try? payload.write(to: outURL)
            case 8:  // deflate — inflate using Compression framework
                if let inflated = inflate(Data(payload), expectedSize: uncompSize) {
                    try? inflated.write(to: outURL)
                }
            default:
                break
            }
        }
    }

    private static func inflate(_ input: Data, expectedSize: Int) -> Data? {
        // Prepend zlib header (0x78 0x9C) that Compression framework expects
        var zlib = Data([0x78, 0x9C]) + input
        var output = Data(count: max(expectedSize, 1))
        let written = zlib.withUnsafeBytes { src in
            output.withUnsafeMutableBytes { dst in
                compression_decode_buffer(
                    dst.baseAddress!.assumingMemoryBound(to: UInt8.self), output.count,
                    src.baseAddress!.assumingMemoryBound(to: UInt8.self), zlib.count,
                    nil, COMPRESSION_ZLIB
                )
            }
        }
        guard written > 0 else { return nil }
        return output.prefix(written)
    }
}

private extension Data {
    func le16(at offset: Int) -> UInt16 {
        guard offset + 2 <= count else { return 0 }
        return UInt16(self[offset]) | (UInt16(self[offset + 1]) << 8)
    }
    func le32(at offset: Int) -> UInt32 {
        guard offset + 4 <= count else { return 0 }
        return UInt32(self[offset])
            | (UInt32(self[offset + 1]) << 8)
            | (UInt32(self[offset + 2]) << 16)
            | (UInt32(self[offset + 3]) << 24)
    }
}
