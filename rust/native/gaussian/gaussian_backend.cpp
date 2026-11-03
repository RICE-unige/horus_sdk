#include "../core/remote_gaussian_backend.h"
#include "Ops.h"
#include <ATen/ATen.h>
#include <ATen/cuda/CUDAContext.h>
#include <c10/core/InferenceMode.h>
#include <c10/cuda/CUDAGuard.h>
#include <c10/cuda/CUDAStream.h>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>

namespace {
thread_local std::string last_error;
template <class F> int guarded(F fn) noexcept {
    try {
        fn();
        last_error.clear();
        return 0;
    } catch (const std::exception &e) {
        last_error = e.what();
    } catch (...) {
        last_error = "Unknown native gsplat error";
    }
    return -1;
}
struct Renderer {
    at::Tensor means, quats, scales, opacity, sh;
    uint32_t degree;
    c10::cuda::CUDAStream stream{c10::cuda::getStreamFromPool()};
};
at::Tensor upload(const float *p, at::IntArrayRef shape) {
    return at::from_blob(const_cast<float *>(p), shape, at::TensorOptions().dtype(at::kFloat))
        .to(at::Device(at::kCUDA));
}
} // namespace
extern "C" {
uint32_t horus_gaussian_abi_version() { return 1; }
const char *horus_gaussian_last_error() { return last_error.c_str(); }
int horus_gaussian_create(size_t count, uint32_t degree, const float *means, const float *quats,
                          const float *scales, const float *opacity, const float *sh, void **out) {
    if (out)
        *out = nullptr;
    return guarded([&] {
        TORCH_CHECK(out && count && count <= INT32_MAX && degree <= 4 && means && quats && scales &&
                        opacity && sh,
                    "Invalid Gaussian inputs");
        c10::InferenceMode inference;
        auto r = std::make_unique<Renderer>();
        c10::cuda::CUDAStreamGuard guard(r->stream);
        r->degree = degree;
        auto n = static_cast<int64_t>(count), k = static_cast<int64_t>((degree + 1) * (degree + 1));
        r->means = upload(means, {n, 3});
        r->quats = upload(quats, {n, 4});
        r->scales = upload(scales, {n, 3});
        r->opacity = upload(opacity, {n});
        r->sh = upload(sh, {n, k, 3});
        r->stream.synchronize();
        *out = r.release();
    });
}
static int render_impl(void *handle, const float *view_data, const float *k_data, uint32_t views,
                       uint32_t width, uint32_t height, float near_m, float far_m, uint8_t *rgb,
                       float *depth, float *elapsed, bool device) {
    return guarded([&] {
        TORCH_CHECK(handle && view_data && k_data && rgb && depth && elapsed && views &&
                        views <= 12 && width && height,
                    "Invalid Gaussian output");
        auto &r = *static_cast<Renderer *>(handle);
        c10::InferenceMode inference;
        c10::cuda::CUDAStreamGuard guard(r.stream);
        const auto start = std::chrono::steady_clock::now();
        auto vm = upload(view_data, {views, 4, 4}), ks = upload(k_data, {views, 3, 3});
        auto [indptr, batch, cam, gid, radii, xy, z, conics, comp] =
            gsplat::projection_ewa_3dgs_packed_fwd(
                r.means, c10::nullopt, r.quats, r.scales, c10::nullopt, vm, ks, width, height, .3f,
                near_m, far_m, 0, true, gsplat::CameraModelType::PINHOLE);
        auto ids = gid.to(at::kLong), cameras = cam.to(at::kLong);
        // R is orthogonal (including the OpenCV Y reflection): camera center=-R^T t.
        auto rot = vm.slice(1, 0, 3).slice(2, 0, 3);
        auto pos = -at::matmul(rot.transpose(1, 2), vm.slice(1, 0, 3).slice(2, 3, 4)).squeeze(-1);
        auto dirs = r.means.index_select(0, ids) - pos.index_select(0, cameras);
        auto colors =
            (gsplat::spherical_harmonics_fwd(r.degree, dirs.contiguous(), r.sh.index_select(0, ids),
                                             radii.gt(0).all(-1).contiguous()) +
             .5)
                .clamp_min(0);
        colors = at::cat({colors, z.unsqueeze(-1)}, -1).contiguous();
        auto opacity = (r.opacity.index_select(0, ids) * comp).contiguous();
        const uint32_t tw = (width + 15) / 16, th = (height + 15) / 16;
        auto [tiles, isects, flatten] =
            gsplat::intersect_tile(xy, radii, z, cam, gid, views, 16, tw, th, true, false);
        auto offsets = gsplat::intersect_offset(isects, views, tw, th);
        auto [pixels, alpha, last] =
            gsplat::rasterize_to_pixels_3dgs_fwd(xy, conics, colors, opacity, c10::nullopt,
                                                 c10::nullopt, width, height, 16, offsets, flatten);
        auto rgb_tensor =
            pixels.slice(-1, 0, 3).clamp(0, 1).mul(255).round().to(at::kByte).contiguous();
        auto d = (pixels.select(-1, 3) / alpha.select(-1, 0).clamp_min(1e-10)).contiguous();
        d = at::where(alpha.select(-1, 0).ge(.01) & at::isfinite(d), d, at::full_like(d, INFINITY));
        if (device) {
            auto out_rgb = at::from_blob(rgb, rgb_tensor.sizes(), rgb_tensor.options());
            auto out_depth = at::from_blob(depth, d.sizes(), d.options());
            out_rgb.copy_(rgb_tensor);
            out_depth.copy_(d);
        } else {
            auto host_rgb = rgb_tensor.cpu().contiguous(), host_depth = d.cpu().contiguous();
            std::memcpy(rgb, host_rgb.data_ptr(), host_rgb.numel());
            std::memcpy(depth, host_depth.data_ptr(), host_depth.numel() * sizeof(float));
        }
        r.stream.synchronize();
        *elapsed =
            std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start)
                .count();
    });
}
int horus_gaussian_render(void *p, const float *v, const float *k, uint32_t n, uint32_t w,
                          uint32_t h, float near, float far, uint8_t *rgb, float *depth,
                          float *ms) {
    return render_impl(p, v, k, n, w, h, near, far, rgb, depth, ms, false);
}
int horus_gaussian_render_device(void *p, const float *v, const float *k, uint32_t n, uint32_t w,
                                 uint32_t h, float near, float far, uint8_t *rgb, float *depth,
                                 float *ms) {
    return render_impl(p, v, k, n, w, h, near, far, rgb, depth, ms, true);
}
void horus_gaussian_destroy(void *p) { delete static_cast<Renderer *>(p); }
}
