#import "RigCalibratorBridge.h"
#import <opencv2/opencv.hpp>
#import <opencv2/calib3d.hpp>
#import <opencv2/features.hpp>

#include <cxxabi.h>
#include <dlfcn.h>
#include <exception>
#include <execinfo.h>
#include <mach-o/dyld.h>
#include <mach-o/loader.h>
#include <mach/mach.h>
#include <pthread.h>
#include <signal.h>
#include <string>
#include <sys/sysctl.h>
#include <sys/ucontext.h>
#include <sys/utsname.h>
#include <typeinfo>
#include <unistd.h>
#include <unordered_map>
#include <vector>
#if __has_feature(ptrauth_calls)
#include <ptrauth.h>
#endif

// SIFT features, triangulation and PnP for RigCalibrator.swift, a check
// of which binary the OpenCV functions are linked from, and crash capture
// into the export trace. OpenCV calls here used to crash because part of
// them were linked from INSCoreMedia's OpenCV 4.7 (see project.yml).

// Reproject error for a 3D point through a 3×4 projection matrix.
static float reprojectError(double X, double Y, double Z,
                            const double *P,
                            float obs_x, float obs_y) {
    double px = P[0]*X + P[1]*Y + P[2]*Z  + P[3];
    double py = P[4]*X + P[5]*Y + P[6]*Z  + P[7];
    double pw = P[8]*X + P[9]*Y + P[10]*Z + P[11];
    if (std::abs(pw) < 1e-8) return 1e6f;
    float dx = (float)(px / pw) - obs_x;
    float dy = (float)(py / pw) - obs_y;
    return std::sqrt(dx * dx + dy * dy);
}

// MARK: - Crash capture
//
// Writes what ends the app during an export into the export trace. iOS
// leaves no crash report for these exports, so this is the only record of
// the signal and where it happened. The signal handler only calls write(2),
// dladdr and pthread getters. Earlier handlers are restored and run after
// ours, so the system report (if any) is still written.

namespace {

volatile int gCrashFD = -1;
const int kCrashSignals[] = {SIGSEGV, SIGBUS, SIGILL, SIGFPE, SIGABRT, SIGTRAP, SIGSYS};
const int kCrashSignalCount = (int)(sizeof(kCrashSignals) / sizeof(kCrashSignals[0]));
struct sigaction gPrevActions[sizeof(kCrashSignals) / sizeof(kCrashSignals[0])];
bool gSignalsInstalled = false;
std::terminate_handler gPrevTerminate = nullptr;
NSUncaughtExceptionHandler *gPrevObjCHandler = nullptr;
bool gExceptionHandlersInstalled = false;
bool gAtExitRegistered = false;
uintptr_t gMainImageBase = 0;

void cdWrite(const char *s) {
    int fd = gCrashFD;
    if (fd < 0 || s == nullptr) return;
    size_t n = strlen(s);
    while (n > 0) {
        ssize_t w = write(fd, s, n);
        if (w <= 0) return;
        s += w;
        n -= (size_t)w;
    }
}

void cdWriteHex(uintptr_t v) {
    char buf[19];
    buf[0] = '0';
    buf[1] = 'x';
    for (int i = 0; i < 16; i++) {
        unsigned nib = (unsigned)((v >> ((15 - i) * 4)) & 0xF);
        buf[2 + i] = (char)(nib < 10 ? '0' + nib : 'a' + (nib - 10));
    }
    buf[18] = '\0';
    cdWrite(buf);
}

void cdWriteDec(long v) {
    char buf[24];
    int i = 22;
    buf[23] = '\0';
    bool neg = v < 0;
    unsigned long u = neg ? (unsigned long)(-(v + 1)) + 1UL : (unsigned long)v;
    do {
        buf[i--] = (char)('0' + (u % 10));
        u /= 10;
    } while (u > 0 && i > 0);
    if (neg) buf[i--] = '-';
    cdWrite(&buf[i + 1]);
}

const char *cdSignalName(int sig) {
    switch (sig) {
        case SIGSEGV: return "SIGSEGV";
        case SIGBUS: return "SIGBUS";
        case SIGILL: return "SIGILL";
        case SIGFPE: return "SIGFPE";
        case SIGABRT: return "SIGABRT";
        case SIGTRAP: return "SIGTRAP";
        case SIGSYS: return "SIGSYS";
        default: return "signal";
    }
}

uintptr_t cdStripPAC(uintptr_t p) {
#if __has_feature(ptrauth_calls)
    return (uintptr_t)ptrauth_strip((void *)p, ptrauth_key_return_address);
#else
    return p;
#endif
}

/// One frame: address, image, offset in the image (for atos with the
/// dSYM), and the symbol name for system libraries.
void cdWriteFrame(int index, uintptr_t addr) {
    cdWrite("  #");
    cdWriteDec(index);
    cdWrite(" ");
    cdWriteHex(addr);
    Dl_info info;
    if (addr != 0 && dladdr((const void *)addr, &info) != 0) {
        const char *slash = info.dli_fname ? strrchr(info.dli_fname, '/') : nullptr;
        cdWrite(" ");
        cdWrite(slash ? slash + 1 : (info.dli_fname ? info.dli_fname : "?"));
        cdWrite(" +");
        cdWriteHex(addr - (uintptr_t)info.dli_fbase);
        if (info.dli_sname && (uintptr_t)info.dli_fbase != gMainImageBase) {
            cdWrite(" ");
            cdWrite(info.dli_sname);
        }
    }
    cdWrite("\n");
}

/// Frame-pointer walk of the interrupted thread, bounded by its stack.
void cdWriteStackWalk(uintptr_t pc, uintptr_t lr, uintptr_t fp) {
    cdWriteFrame(0, pc);
    cdWriteFrame(1, lr);
    pthread_t thread = pthread_self();
    uintptr_t hi = (uintptr_t)pthread_get_stackaddr_np(thread);
    uintptr_t lo = hi - (uintptr_t)pthread_get_stacksize_np(thread);
    uintptr_t prev = 0;
    int index = 2;
    while (index < 48 && fp >= lo && fp + 16 <= hi && (fp & 0x7) == 0 && fp > prev) {
        const uintptr_t *frame = (const uintptr_t *)fp;
        uintptr_t next = frame[0];
        uintptr_t ret = cdStripPAC(frame[1]);
        if (ret == 0) break;
        cdWriteFrame(index++, ret);
        prev = fp;
        fp = next;
    }
}

void cdWriteBacktraceHere() {
    void *frames[64];
    int n = backtrace(frames, 64);
    int fd = gCrashFD;
    if (fd >= 0 && n > 0) backtrace_symbols_fd(frames, n, fd);
}

void cdSignalHandler(int sig, siginfo_t *info, void *context) {
    cdWrite("\n!!! CRASH: ");
    cdWrite(cdSignalName(sig));
    cdWrite(" (");
    cdWriteDec(sig);
    cdWrite(") code ");
    cdWriteDec(info ? (long)info->si_code : 0);
    cdWrite(" fault address ");
    cdWriteHex(info ? (uintptr_t)info->si_addr : 0);
    cdWrite(" app image base ");
    cdWriteHex(gMainImageBase);
    cdWrite("\n");
#if defined(__arm64__)
    ucontext_t *uc = (ucontext_t *)context;
    if (uc != nullptr && uc->uc_mcontext != nullptr) {
#if defined(__darwin_arm_thread_state64_get_pc)
        uintptr_t pc = (uintptr_t)__darwin_arm_thread_state64_get_pc(uc->uc_mcontext->__ss);
        uintptr_t lr = (uintptr_t)__darwin_arm_thread_state64_get_lr(uc->uc_mcontext->__ss);
        uintptr_t fp = (uintptr_t)__darwin_arm_thread_state64_get_fp(uc->uc_mcontext->__ss);
        uintptr_t sp = (uintptr_t)__darwin_arm_thread_state64_get_sp(uc->uc_mcontext->__ss);
#else
        uintptr_t pc = (uintptr_t)uc->uc_mcontext->__ss.__pc;
        uintptr_t lr = (uintptr_t)uc->uc_mcontext->__ss.__lr;
        uintptr_t fp = (uintptr_t)uc->uc_mcontext->__ss.__fp;
        uintptr_t sp = (uintptr_t)uc->uc_mcontext->__ss.__sp;
#endif
        pc = cdStripPAC(pc);
        lr = cdStripPAC(lr);
        cdWrite("  pc ");
        cdWriteHex(pc);
        cdWrite(" lr ");
        cdWriteHex(lr);
        cdWrite(" fp ");
        cdWriteHex(fp);
        cdWrite(" sp ");
        cdWriteHex(sp);
        cdWrite(" thread stack ");
        cdWriteDec((long)(pthread_get_stacksize_np(pthread_self()) / 1024));
        cdWrite(" KB\n");
        cdWriteStackWalk(pc, lr, fp);
    }
#else
    (void)context;
    cdWriteBacktraceHere();
#endif
    // Hand the signal to the earlier handler, or to the default action that
    // writes the system crash report.
    for (int i = 0; i < kCrashSignalCount; i++) {
        if (kCrashSignals[i] == sig) {
            sigaction(sig, &gPrevActions[i], nullptr);
            break;
        }
    }
    // A fault re-executes the faulting instruction on return and reaches
    // the restored handler; a raised signal has to be raised again.
    if (sig == SIGABRT || sig == SIGSYS || (info != nullptr && info->si_code == SI_USER)) {
        raise(sig);
    }
}

void cdTerminateHandler() {
    cdWrite("\n!!! CRASH: std::terminate");
    std::exception_ptr ep = std::current_exception();
    if (ep) {
        try {
            std::rethrow_exception(ep);
        } catch (const std::exception &e) {
            int status = 0;
            char *name = abi::__cxa_demangle(typeid(e).name(), nullptr, nullptr, &status);
            cdWrite(", uncaught C++ exception ");
            cdWrite(name ? name : typeid(e).name());
            cdWrite(": ");
            cdWrite(e.what());
            free(name);
        } catch (...) {
            cdWrite(", uncaught C++ exception of unknown type");
        }
    }
    cdWrite("\n");
    cdWriteBacktraceHere();
    if (gPrevTerminate) gPrevTerminate();
    abort();
}

void cdObjCExceptionHandler(NSException *exception) {
    cdWrite("\n!!! CRASH: uncaught Objective-C exception ");
    cdWrite(exception.name.UTF8String);
    cdWrite(": ");
    cdWrite(exception.reason.UTF8String);
    cdWrite("\n");
    for (NSString *frame in exception.callStackSymbols) {
        cdWrite("  ");
        cdWrite(frame.UTF8String);
        cdWrite("\n");
    }
    if (gPrevObjCHandler) gPrevObjCHandler(exception);
}

void cdAtExit() {
    if (gCrashFD < 0) return;
    cdWrite("\n!!! exit() was called during the export\n");
    cdWriteBacktraceHere();
}

NSString *cdDescribeFunction(void *fn) {
    if (fn == nullptr) return @"none";
    Dl_info info;
    if (dladdr(fn, &info) != 0 && info.dli_fname != nullptr) {
        const char *slash = strrchr(info.dli_fname, '/');
        return [NSString stringWithFormat:@"%s (%s)",
                slash ? slash + 1 : info.dli_fname,
                info.dli_sname ? info.dli_sname : "?"];
    }
    return [NSString stringWithFormat:@"%p", fn];
}

NSString *cdSysctlValue(const char *name) {
    int64_t v = 0;
    size_t len = sizeof(v);
    if (sysctlbyname(name, &v, &len, nullptr, 0) != 0) return @"n/a";
    if (len == sizeof(int32_t)) {
        int32_t v32 = 0;
        memcpy(&v32, &v, sizeof(v32));
        return [NSString stringWithFormat:@"%d", v32];
    }
    return [NSString stringWithFormat:@"%lld", (long long)v];
}

/// Mach exception handlers registered for the app. A crash reporter that
/// takes crashes here (before they become signals) can end the app without
/// a system crash report and without running signal handlers.
NSString *cdMachExceptionPorts() {
    exception_mask_t masks[EXC_TYPES_COUNT];
    mach_port_t ports[EXC_TYPES_COUNT];
    exception_behavior_t behaviors[EXC_TYPES_COUNT];
    thread_state_flavor_t flavors[EXC_TYPES_COUNT];
    mach_msg_type_number_t count = EXC_TYPES_COUNT;
    exception_mask_t want = EXC_MASK_BAD_ACCESS | EXC_MASK_BAD_INSTRUCTION
        | EXC_MASK_ARITHMETIC | EXC_MASK_SOFTWARE | EXC_MASK_BREAKPOINT;
    kern_return_t kr = task_get_exception_ports(
        mach_task_self(), want, masks, &count, ports, behaviors, flavors);
    if (kr != KERN_SUCCESS) {
        return [NSString stringWithFormat:@"task exception ports: error %d", (int)kr];
    }
    NSMutableString *set = [NSMutableString string];
    for (mach_msg_type_number_t i = 0; i < count; i++) {
        if (MACH_PORT_VALID(ports[i])) {
            [set appendFormat:@" mask 0x%x", (unsigned)masks[i]];
        }
    }
    return set.length > 0
        ? [@"task exception ports set:" stringByAppendingString:set]
        : @"task exception ports: none";
}

/// Main image name, load address, slide and UUID (to pick the dSYM).
NSString *cdMainImageDescription() {
    const struct mach_header *mh = _dyld_get_image_header(0);
    gMainImageBase = (uintptr_t)mh;
    intptr_t slide = _dyld_get_image_vmaddr_slide(0);
    const char *path = _dyld_get_image_name(0);
    NSString *uuid = @"?";
    if (mh != nullptr && mh->magic == MH_MAGIC_64) {
        const struct mach_header_64 *mh64 = (const struct mach_header_64 *)mh;
        const uint8_t *p = (const uint8_t *)(mh64 + 1);
        for (uint32_t i = 0; i < mh64->ncmds; i++) {
            const struct load_command *lc = (const struct load_command *)p;
            if (lc->cmd == LC_UUID) {
                const struct uuid_command *u = (const struct uuid_command *)p;
                uuid = [[NSUUID alloc] initWithUUIDBytes:u->uuid].UUIDString;
                break;
            }
            p += lc->cmdsize;
        }
    }
    const char *slash = path ? strrchr(path, '/') : nullptr;
    return [NSString stringWithFormat:@"app image %s base 0x%lx slide 0x%lx uuid %@",
            slash ? slash + 1 : (path ? path : "?"),
            (unsigned long)gMainImageBase, (unsigned long)slide, uuid];
}

// MARK: - OpenCV linking check
//
// INSCoreMedia.framework (Insta360 SDK) exports its own OpenCV 4.7.0. When
// it came before opencv2 on the link line, the linker bound part of the
// app's OpenCV calls to it, and those functions read cv::Mat (OpenCV 5
// layout) at the wrong offsets. dladdr tells which binary each function
// below was bound to.

/// Code address of a function, or of a non-virtual member function: a
/// pointer to member function starts with the code address (Itanium C++
/// ABI, as used on arm64).
template <typename F>
const void *cvCodeAddress(F f) {
    static_assert(sizeof(F) >= sizeof(void *), "unexpected function pointer size");
    const void *p = nullptr;
    memcpy(&p, &f, sizeof(p));
#if __has_feature(ptrauth_calls)
    p = ptrauth_strip(const_cast<void *>(p), ptrauth_key_function_pointer);
#endif
    return p;
}

struct CVBinding {
    const char *name;
    const void *address;
};

/// OpenCV functions this file calls, plus functions OpenCV calls inside
/// them: the NAryMatIterator functions inside cv::Mat::setTo (where the
/// crash happened) and GaussianBlur inside SIFT.
std::vector<CVBinding> cvBindings() {
    return {
        {"cv::Mat::setTo", cvCodeAddress(&cv::Mat::setTo)},
        {"cv::NAryMatIterator::init", cvCodeAddress(&cv::NAryMatIterator::init)},
        {"cv::NAryMatIterator::operator++", cvCodeAddress(
            static_cast<cv::NAryMatIterator &(cv::NAryMatIterator::*)()>(
                &cv::NAryMatIterator::operator++))},
        {"cv::SVD::compute", cvCodeAddress(
            static_cast<void (*)(cv::InputArray, cv::OutputArray, cv::OutputArray,
                                 cv::OutputArray, int)>(&cv::SVD::compute))},
        {"cv::SVDecomp", cvCodeAddress(&cv::SVDecomp)},
        {"cv::Rodrigues", cvCodeAddress(&cv::Rodrigues)},
        {"cv::solvePnPRansac", cvCodeAddress(
            static_cast<bool (*)(cv::InputArray, cv::InputArray, cv::InputArray,
                                 cv::InputArray, cv::OutputArray, cv::OutputArray,
                                 bool, int, float, double, cv::OutputArray, int)>(
                &cv::solvePnPRansac))},
        {"cv::GaussianBlur", cvCodeAddress(&cv::GaussianBlur)},
        {"cv::SIFT::create", cvCodeAddress(
            static_cast<cv::Ptr<cv::SIFT> (*)(int, int, double, double, double, int, bool)>(
                &cv::SIFT::create))},
    };
}

/// Load address of the binary that contains this file's code (the app).
uintptr_t cvBridgeImageBase() {
    Dl_info info;
    if (dladdr(cvCodeAddress(&reprojectError), &info) == 0) return 0;
    return (uintptr_t)info.dli_fbase;
}

/// File name of the binary that contains `address`, or "?".
NSString *cvImageName(const void *address) {
    Dl_info info;
    if (address == nullptr || dladdr(address, &info) == 0 || info.dli_fname == nullptr) {
        return @"?";
    }
    const char *slash = strrchr(info.dli_fname, '/');
    NSString *name = [NSString stringWithUTF8String:slash ? slash + 1 : info.dli_fname];
    return name ?: @"?";
}

} // namespace

@implementation RigCalibratorBridge

+ (nullable NSArray<NSNumber *> *)
    triangulatePoint:(NSArray<NSArray<NSNumber *> *> *)observations
         projMatrices:(NSDictionary<NSNumber *, NSData *> *)projMats
            maxReproj:(float)maxReproj
          minObserved:(int)minObserved
{
  try {
    if (observations.count < 2) return nil;

    // Collect valid observations (those with a projection matrix).
    // Copy the 12 doubles: pd.bytes is only valid while pd is alive.
    struct Obs { int imgId; float x, y; double P[12]; };
    std::vector<Obs> obs;
    for (NSArray *o in observations) {
        if (o.count < 3) continue;
        int imgId = [o[0] intValue];
        NSData *pd = projMats[@(imgId)];
        if (!pd || pd.length < 12 * sizeof(double)) continue;
        Obs ob;
        ob.imgId = imgId;
        ob.x = [o[1] floatValue];
        ob.y = [o[2] floatValue];
        memcpy(ob.P, pd.bytes, 12 * sizeof(double));
        obs.push_back(ob);
    }
    if ((int)obs.size() < 2) return nil;

    // Pick the pair with widest baseline (first and last by imgId)
    int iA = 0, iB = (int)obs.size() - 1;
    const double *PA = obs[iA].P;
    const double *PB = obs[iB].P;

    // DLT 4×4 via SVD
    double A[16];
    auto row = [](double *dst, float x, float y,
                  const double *P, int r) {
        for (int c = 0; c < 4; c++)
            dst[r * 4 + c] = x * P[8 + c] - P[r * 4 + c];
    };
    row(A, obs[iA].x, obs[iA].y, PA, 0);
    row(A, obs[iA].x, obs[iA].y, PA, 1);
    row(A, obs[iB].x, obs[iB].y, PB, 0);
    row(A, obs[iB].x, obs[iB].y, PB, 1);

    // Manually overwrite rows — the lambda wrote row 0/1 for A, row 0/1 for B
    // Fix: A rows are (x*P2 - P0, y*P2 - P1) for each view
    for (int c = 0; c < 4; c++) {
        A[0 * 4 + c] =
            (double)obs[iA].x * PA[8 + c] - PA[0 * 4 + c];
        A[1 * 4 + c] =
            (double)obs[iA].y * PA[8 + c] - PA[1 * 4 + c];
        A[2 * 4 + c] =
            (double)obs[iB].x * PB[8 + c] - PB[0 * 4 + c];
        A[3 * 4 + c] =
            (double)obs[iB].y * PB[8 + c] - PB[1 * 4 + c];
    }

    cv::Mat Am(4, 4, CV_64F, A);
    cv::Mat w, u, vt;
    cv::SVD::compute(Am, w, u, vt, cv::SVD::FULL_UV);
    double hw = vt.at<double>(3, 3);
    if (std::abs(hw) < 1e-8) return nil;

    double X = vt.at<double>(3, 0) / hw;
    double Y = vt.at<double>(3, 1) / hw;
    double Z = vt.at<double>(3, 2) / hw;

    // Positive depth check for both views
    double dA = PA[8]*X + PA[9]*Y + PA[10]*Z + PA[11];
    double dB = PB[8]*X + PB[9]*Y + PB[10]*Z + PB[11];
    if (dA <= 0 || dB <= 0) return nil;

    // Reprojection check across ALL observations
    float totalErr = 0;
    int valid = 0;
    for (auto &o : obs) {
        float err = reprojectError(X, Y, Z, o.P, o.x, o.y);
        if (err <= maxReproj) {
            totalErr += err;
            valid++;
        }
    }
    if (valid < minObserved) return nil;

    float meanErr = totalErr / (float)valid;
    return @[@(X), @(Y), @(Z), @(meanErr), @(valid)];
  } catch (const std::exception &e) {
    NSLog(@"[RigCalBridge] triangulate: %s", e.what());
    return nil;
  } catch (...) {
    NSLog(@"[RigCalBridge] triangulate: unknown exception");
    return nil;
  }
}

+ (nullable NSDictionary *)
    solvePnPRansac:(NSArray<NSArray<NSNumber *> *> *)points3D
          points2D:(NSArray<NSArray<NSNumber *> *> *)points2D
                fx:(double)fx fy:(double)fy
                cx:(double)cx cy:(double)cy
        minInliers:(int)minInliers
{
  // The whole body is inside try: a cv::Exception that escapes this
  // method unwinds through Swift and aborts the app.
  try {
    int n = (int)points3D.count;
    if (n < 4 || (int)points2D.count != n) return nil;

    std::vector<cv::Point3d> obj;
    std::vector<cv::Point2d> img;
    obj.reserve(n);
    img.reserve(n);
    for (int i = 0; i < n; i++) {
        NSArray<NSNumber *> *p3 = points3D[i];
        NSArray<NSNumber *> *p2 = points2D[i];
        if (p3.count < 3 || p2.count < 2) return nil;
        obj.emplace_back([p3[0] doubleValue], [p3[1] doubleValue],
                         [p3[2] doubleValue]);
        img.emplace_back([p2[0] doubleValue], [p2[1] doubleValue]);
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cv::Mat dist;  // no distortion
    cv::Mat rvec, tvec;
    // std::vector gives the right inlier count on OpenCV 4 (Nx1 Mat)
    // and OpenCV 5 (1-D Mat, where Mat::rows is always 1).
    std::vector<int> inliers;

    bool ok = cv::solvePnPRansac(
        obj, img, K, dist, rvec, tvec,
        false,         // useExtrinsicGuess
        100,           // iterationsCount
        8.0,           // reprojectionError pixels
        0.99,          // confidence
        inliers,
        cv::SOLVEPNP_ITERATIVE);
    if (!ok || (int)inliers.size() < minInliers) return nil;

    // Refine with LM on inliers
    std::vector<cv::Point3d> objIn;
    std::vector<cv::Point2d> imgIn;
    objIn.reserve(inliers.size());
    imgIn.reserve(inliers.size());
    for (int idx : inliers) {
        if (idx < 0 || idx >= n) continue;
        objIn.push_back(obj[idx]);
        imgIn.push_back(img[idx]);
    }
    int inlierCount = (int)objIn.size();
    if (inlierCount < minInliers) return nil;
    try {
        cv::solvePnPRefineLM(objIn, imgIn, K, dist, rvec, tvec);
    } catch (...) {
        // refinement failure is non-fatal
    }

    if (rvec.total() != 3 || tvec.total() != 3) return nil;
    cv::Mat rv, tv;
    rvec.convertTo(rv, CV_64F);
    tvec.convertTo(tv, CV_64F);
    rv = rv.reshape(1, 3);
    tv = tv.reshape(1, 3);

    cv::Mat R;
    cv::Rodrigues(rv, R);
    if (R.rows != 3 || R.cols != 3) return nil;
    R.convertTo(R, CV_64F);

    NSMutableArray *Rarr = [NSMutableArray arrayWithCapacity:9];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            [Rarr addObject:@(R.at<double>(r, c))];

    return @{
        @"R": Rarr,
        @"rvec": @[@(rv.at<double>(0)),
                   @(rv.at<double>(1)),
                   @(rv.at<double>(2))],
        @"tvec": @[@(tv.at<double>(0)),
                   @(tv.at<double>(1)),
                   @(tv.at<double>(2))],
        @"inlierCount": @(inlierCount),
    };
  } catch (const std::exception &e) {
    NSLog(@"[RigCalBridge] solvePnPRansac: %s", e.what());
    return nil;
  } catch (...) {
    NSLog(@"[RigCalBridge] solvePnPRansac: unknown exception");
    return nil;
  }
}

// MARK: - OpenCV linking check

+ (NSString *)openCVBindingReport
{
    NSMutableArray<NSString *> *parts = [NSMutableArray array];
    for (const CVBinding &b : cvBindings()) {
        [parts addObject:[NSString stringWithFormat:@"%s %@", b.name, cvImageName(b.address)]];
    }
    return [parts componentsJoinedByString:@"; "];
}

+ (BOOL)openCVLinkedFromApp
{
    // true/false rather than YES/NO: opencv2/opencv.hpp includes
    // stitching/detail/exposure_compensate.hpp, which does #undef NO.
    uintptr_t base = cvBridgeImageBase();
    if (base == 0) return false;
    for (const CVBinding &b : cvBindings()) {
        Dl_info info;
        if (b.address == nullptr || dladdr(b.address, &info) == 0
            || (uintptr_t)info.dli_fbase != base) {
            return false;
        }
    }
    return true;
}

// MARK: - Crash diagnostics

+ (NSString *)installCrashCaptureWithFileDescriptor:(int)fd
{
    NSMutableArray<NSString *> *lines = [NSMutableArray array];
    struct utsname u;
    uname(&u);
    [lines addObject:[NSString stringWithFormat:@"device %s, %@", u.machine,
                      NSProcessInfo.processInfo.operatingSystemVersionString]];
    [lines addObject:[NSString stringWithFormat:
        @"CPU: DotProd %@, FP16 %@, BF16 %@, I8MM %@, SME %@",
        cdSysctlValue("hw.optional.arm.FEAT_DotProd"),
        cdSysctlValue("hw.optional.arm.FEAT_FP16"),
        cdSysctlValue("hw.optional.arm.FEAT_BF16"),
        cdSysctlValue("hw.optional.arm.FEAT_I8MM"),
        cdSysctlValue("hw.optional.arm.FEAT_SME")]];
    [lines addObject:cdMainImageDescription()];

    // Handlers someone else installed decide whether iOS still writes its
    // own crash report, so list them.
    NSMutableArray<NSString *> *earlier = [NSMutableArray array];
    for (int i = 0; i < kCrashSignalCount; i++) {
        struct sigaction cur;
        if (sigaction(kCrashSignals[i], nullptr, &cur) != 0) continue;
        void *h = reinterpret_cast<void *>(cur.sa_handler);
        if (h == reinterpret_cast<void *>(&cdSignalHandler)) continue;
        NSString *d = h == reinterpret_cast<void *>(SIG_DFL) ? @"default"
                    : h == reinterpret_cast<void *>(SIG_IGN) ? @"ignored"
                    : cdDescribeFunction(h);
        [earlier addObject:[NSString stringWithFormat:@"%s %@",
                            cdSignalName(kCrashSignals[i]), d]];
    }
    [lines addObject:[@"signal handlers before ours: "
                      stringByAppendingString:[earlier componentsJoinedByString:@", "]]];
    std::terminate_handler term = std::get_terminate();
    NSUncaughtExceptionHandler *objcHandler = NSGetUncaughtExceptionHandler();
    [lines addObject:[NSString stringWithFormat:
        @"terminate handler before ours: %@; Objective-C exception handler: %@",
        term == &cdTerminateHandler ? @"ours" : cdDescribeFunction(reinterpret_cast<void *>(term)),
        objcHandler == &cdObjCExceptionHandler
            ? @"ours" : cdDescribeFunction(reinterpret_cast<void *>(objcHandler))]];
    [lines addObject:cdMachExceptionPorts()];

    gCrashFD = fd;
    if (!gSignalsInstalled) {
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_sigaction = cdSignalHandler;
        sa.sa_flags = SA_SIGINFO | SA_ONSTACK;
        sigemptyset(&sa.sa_mask);
        for (int i = 0; i < kCrashSignalCount; i++) {
            sigaction(kCrashSignals[i], &sa, &gPrevActions[i]);
        }
        gSignalsInstalled = true;
    }
    if (!gExceptionHandlersInstalled) {
        gPrevTerminate = std::set_terminate(&cdTerminateHandler);
        gPrevObjCHandler = NSGetUncaughtExceptionHandler();
        NSSetUncaughtExceptionHandler(&cdObjCExceptionHandler);
        gExceptionHandlersInstalled = true;
    }
    if (!gAtExitRegistered) {
        atexit(&cdAtExit);
        gAtExitRegistered = true;
    }
    return [lines componentsJoinedByString:@"\n"];
}

+ (void)stopCrashCapture
{
    gCrashFD = -1;
    if (gSignalsInstalled) {
        for (int i = 0; i < kCrashSignalCount; i++) {
            sigaction(kCrashSignals[i], &gPrevActions[i], nullptr);
        }
        gSignalsInstalled = false;
    }
    if (gExceptionHandlersInstalled) {
        std::set_terminate(gPrevTerminate);
        NSSetUncaughtExceptionHandler(gPrevObjCHandler);
        gExceptionHandlersInstalled = false;
    }
}

+ (void)useAlternateSignalStackOnCurrentThread
{
    // Exports can run on a reused thread; keep a stack set up earlier
    // instead of replacing (and leaking) it.
    stack_t current;
    if (sigaltstack(nullptr, &current) == 0 && !(current.ss_flags & SS_DISABLE)) return;
    const size_t size = 256 * 1024;
    stack_t ss;
    memset(&ss, 0, sizeof(ss));
    ss.ss_sp = malloc(size);
    if (ss.ss_sp == nullptr) return;
    ss.ss_size = size;
    ss.ss_flags = 0;
    if (sigaltstack(&ss, nullptr) != 0) free(ss.ss_sp);
}

@end

// MARK: - SIFT features for rig calibration

@implementation RigSIFTFeatures {
    int _maxFeatures;
    std::unordered_map<int, std::vector<cv::Point2f>> _points;
    std::unordered_map<int, cv::Mat> _descriptors;
}

- (instancetype)initWithMaxFeatures:(int)maxFeatures
{
    self = [super init];
    if (self) _maxFeatures = std::max(0, maxFeatures);
    return self;
}

- (int)addGray:(const uint8_t *)gray
         width:(int)width
        height:(int)height
       imageId:(int)imageId
{
    try {
        if (gray == nullptr || width < 16 || height < 16) return -1;
        // Wraps the caller's buffer for this call only; the descriptors
        // OpenCV returns own their memory.
        cv::Mat img(height, width, CV_8UC1, const_cast<uint8_t *>(gray));
        // 8-bit descriptors: OpenCV's float SIFT values are whole numbers
        // from 0 to 255, so this keeps them exactly in a quarter of the
        // memory.
        cv::Ptr<cv::SIFT> sift = cv::SIFT::create(
            _maxFeatures, 3, 0.04, 10.0, 1.6, CV_8U, false);
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        sift->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
        if ((int)keypoints.size() != descriptors.rows) return -1;
        std::vector<cv::Point2f> points;
        points.reserve(keypoints.size());
        for (const cv::KeyPoint &k : keypoints) points.push_back(k.pt);
        _points[imageId] = std::move(points);
        _descriptors[imageId] = descriptors;
        return descriptors.rows;
    } catch (const std::exception &e) {
        NSLog(@"[RigSIFT] addGray: %s", e.what());
        return -1;
    } catch (...) {
        NSLog(@"[RigSIFT] addGray: unknown exception");
        return -1;
    }
}

- (nullable NSData *)matchQuery:(int)queryId
                          train:(int)trainId
                          ratio:(float)ratio
{
    try {
        auto qd = _descriptors.find(queryId);
        auto td = _descriptors.find(trainId);
        auto qp = _points.find(queryId);
        auto tp = _points.find(trainId);
        if (qd == _descriptors.end() || td == _descriptors.end()
            || qp == _points.end() || tp == _points.end()) return nil;
        if (qd->second.rows < 2 || td->second.rows < 2) return nil;

        cv::BFMatcher matcher(cv::NORM_L2, false);
        std::vector<std::vector<cv::DMatch>> knn;
        matcher.knnMatch(qd->second, td->second, knn, 2);

        const std::vector<cv::Point2f> &qpts = qp->second;
        const std::vector<cv::Point2f> &tpts = tp->second;
        std::vector<float> out;
        out.reserve(knn.size() * 4);
        for (const std::vector<cv::DMatch> &m : knn) {
            if (m.size() < 2 || !(m[0].distance < ratio * m[1].distance)) continue;
            const int qi = m[0].queryIdx;
            const int ti = m[0].trainIdx;
            if (qi < 0 || qi >= (int)qpts.size() || ti < 0 || ti >= (int)tpts.size()) continue;
            out.push_back(qpts[qi].x);
            out.push_back(qpts[qi].y);
            out.push_back(tpts[ti].x);
            out.push_back(tpts[ti].y);
        }
        if (out.empty()) return nil;
        return [NSData dataWithBytes:out.data() length:out.size() * sizeof(float)];
    } catch (const std::exception &e) {
        NSLog(@"[RigSIFT] match: %s", e.what());
        return nil;
    } catch (...) {
        NSLog(@"[RigSIFT] match: unknown exception");
        return nil;
    }
}

- (void)reset
{
    std::unordered_map<int, std::vector<cv::Point2f>>().swap(_points);
    std::unordered_map<int, cv::Mat>().swap(_descriptors);
}

@end
