import Foundation
import ImageIO

/// The EXIF exposure fields the app writes into its images, copied from
/// the photo each image comes from: ARKit's frame metadata for the iPhone
/// image, the INSP for the 360 tiles. A trainer reads each frame's
/// relative exposure from them. 3DGRUT's PPISP post-processing reads
/// ExposureTime, FNumber and ISO (PhotographicSensitivity) with piexif and
/// uses log2(time x ISO / FNumber^2), less its mean over all images
/// (threedgrut/utils/exif.py); images without them get no exposure prior.
enum ExposureMetadata {
    /// The Exif keys copied: the exposure settings and whether they were
    /// automatic, the white balance mode, and the capture time.
    /// ShutterSpeedValue and ApertureValue are left out: the X5 writes
    /// ShutterSpeedValue 0, which a reader without ExposureTime would take
    /// as 1 s.
    static let exifKeys: [String] = [
        kCGImagePropertyExifExposureTime,
        kCGImagePropertyExifFNumber,
        kCGImagePropertyExifISOSpeedRatings,
        kCGImagePropertyExifExposureBiasValue,
        kCGImagePropertyExifExposureProgram,
        kCGImagePropertyExifExposureMode,
        kCGImagePropertyExifWhiteBalance,
        kCGImagePropertyExifDateTimeOriginal,
        kCGImagePropertyExifSubsecTimeOriginal,
    ].map { $0 as String }

    /// The copied part of an Exif dictionary, such as ARFrame.exifData.
    static func subset(_ exif: [String: Any]) -> [String: Any] {
        var out: [String: Any] = [:]
        for key in exifKeys {
            if let value = exif[key] { out[key] = value }
        }
        return out
    }

    /// The copied part of an image file's Exif dictionary (an INSP), or
    /// empty if it has none. Reads the metadata only, not the pixels.
    static func subset(fileAt url: URL) -> [String: Any] {
        guard let source = CGImageSourceCreateWithURL(url as CFURL, nil),
              let props = CGImageSourceCopyPropertiesAtIndex(
                  source, 0, nil) as? [String: Any],
              let exif = props[kCGImagePropertyExifDictionary as String]
                  as? [String: Any]
        else { return [:] }
        return subset(exif)
    }

    /// CGImageDestinationAddImage properties for a JPEG: the quality and,
    /// if there is any, the Exif dictionary.
    static func jpegProperties(
        quality: CGFloat, exif: [String: Any]
    ) -> CFDictionary {
        var props: [String: Any] = [
            kCGImageDestinationLossyCompressionQuality as String: quality,
        ]
        if !exif.isEmpty {
            props[kCGImagePropertyExifDictionary as String] = exif
        }
        return props as CFDictionary
    }

    /// The exposure as text for logs, e.g. "1/25 s, f/2.0, ISO 248".
    static func summary(_ exif: [String: Any]) -> String {
        var parts: [String] = []
        let time = exif[kCGImagePropertyExifExposureTime as String]
            as? NSNumber
        if let t = time?.doubleValue, t > 0 {
            parts.append(t < 1
                ? String(format: "1/%.0f s", 1 / t)
                : String(format: "%.1f s", t))
        }
        let fNumber = exif[kCGImagePropertyExifFNumber as String]
            as? NSNumber
        if let n = fNumber?.doubleValue {
            parts.append(String(format: "f/%.1f", n))
        }
        let isoValue = exif[kCGImagePropertyExifISOSpeedRatings as String]
        if let iso = (isoValue as? [NSNumber])?.first
            ?? (isoValue as? NSNumber)
        {
            parts.append("ISO \(iso.intValue)")
        }
        return parts.isEmpty ? "no exposure EXIF" : parts.joined(separator: ", ")
    }
}
