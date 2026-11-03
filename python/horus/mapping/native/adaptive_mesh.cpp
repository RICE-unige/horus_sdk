// Boundary-locked, attribute-aware coarse geometry; fine geometry is never edited.
#include "meshoptimizer.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>
#include <unordered_map>
#include <vector>

namespace {
using Edges = std::unordered_map<uint64_t, size_t>;

// TwoDiff's residual proves whether a subtraction of source float32s was exact.
double difference_error(double a, double b, double value) {
    double bv = a - value;
    double av = value + bv;
    return (a - av) + (bv - b);
}

bool zero_area(const float* a, const float* b, const float* c) {
    double u[3], v[3];
    bool repeated_ab = true, repeated_ac = true, repeated_bc = true;
    for (size_t i = 0; i < 3; ++i) {
        repeated_ab &= a[i] == b[i];
        repeated_ac &= a[i] == c[i];
        repeated_bc &= b[i] == c[i];
        u[i] = double(b[i]) - a[i];
        v[i] = double(c[i]) - a[i];
    }
    if (repeated_ab || repeated_ac || repeated_bc) return true;
    for (size_t i = 0; i < 3; ++i)
        if (difference_error(b[i], a[i], u[i]) || difference_error(c[i], a[i], v[i]))
            return false;
    // Float32 products cannot under/overflow double; FMA retains their exact residual.
    for (size_t i = 0; i < 3; ++i) {
        size_t j = (i + 1) % 3, k = (i + 2) % 3;
        double p = u[j] * v[k], q = u[k] * v[j];
        if (p != q || std::fma(u[j], v[k], -p) != std::fma(u[k], v[j], -q)) return false;
    }
    return true;
}

uint64_t edge(unsigned int a, unsigned int b) {
    return (uint64_t(std::min(a, b)) << 32) | std::max(a, b);
}

Edges edges(const std::vector<unsigned int>& indices) {
    Edges result;
    for (size_t i = 0; i < indices.size(); i += 3) {
        unsigned int a = indices[i], b = indices[i + 1], c = indices[i + 2];
        if (a == b || b == c || c == a) continue;
        ++result[edge(a, b)];
        ++result[edge(b, c)];
        ++result[edge(c, a)];
    }
    return result;
}

unsigned int root(std::vector<unsigned int>& parents, unsigned int i) {
    while (parents[i] != i) {
        parents[i] = parents[parents[i]];
        i = parents[i];
    }
    return i;
}

unsigned int safe_topology(const std::vector<unsigned int>& before,
                   const std::vector<unsigned int>& after, size_t count,
                   const Edges& original) {
    Edges reduced = edges(after);
    for (const auto& item : original) {
        if (item.second == 2) continue;
        auto found = reduced.find(item.first);
        if (found == reduced.end() || found->second != item.second) return 5;
    }
    for (const auto& item : reduced) {
        if (item.second == 2) continue;
        auto found = original.find(item.first);
        if (found == original.end() || found->second != item.second) return 5;
    }
    // Closed components have no open edges; also verify none disappeared.
    std::vector<unsigned int> parents(count);
    for (size_t i = 0; i < count; ++i) parents[i] = unsigned(i);
    for (size_t i = 0; i < before.size(); i += 3) {
        unsigned int a = root(parents, before[i]);
        for (size_t j = 1; j < 3; ++j)
            parents[root(parents, before[i + j])] = a;
    }
    std::vector<unsigned char> present(count, 0);
    for (unsigned int i : after) present[root(parents, i)] = 1;
    for (unsigned int i : before) if (!present[root(parents, i)]) return 6;
    return 0;
}

bool restore_unsafe_components(const std::vector<unsigned int>& before,
                               std::vector<unsigned int>& after, size_t count,
                               const Edges& original, size_t* diagnostics) {
    std::vector<unsigned int> parents(count);
    for (size_t i = 0; i < count; ++i) parents[i] = unsigned(i);
    for (size_t i = 0; i < before.size(); i += 3) {
        unsigned int a = root(parents, before[i]);
        parents[root(parents, before[i + 1])] = a;
        parents[root(parents, before[i + 2])] = a;
    }
    for (size_t i = 0; i < count; ++i) parents[i] = root(parents, unsigned(i));
    std::vector<size_t> source_count(count, 0), output_count(count, 0);
    std::vector<unsigned char> unsafe(count, 0);
    for (size_t i = 0; i < before.size(); i += 3) ++source_count[parents[before[i]]];
    for (size_t i = 0; i < after.size(); i += 3) {
        unsigned int a = parents[after[i]], b = parents[after[i + 1]], c = parents[after[i + 2]];
        ++output_count[a];
        if (a != b || a != c) unsafe[a] |= 1, unsafe[b] |= 1, unsafe[c] |= 1;
    }
    Edges reduced = edges(after);
    for (const auto& item : original) {
        if (item.second == 2) continue;
        auto found = reduced.find(item.first);
        if (found == reduced.end() || found->second != item.second)
            unsafe[parents[item.first >> 32]] |= 1;
    }
    for (const auto& item : reduced) {
        if (item.second == 2) continue;
        auto found = original.find(item.first);
        if (found == original.end() || found->second != item.second) {
            unsafe[parents[item.first >> 32]] |= 1;
            unsafe[parents[unsigned(item.first)]] |= 1;
        }
    }
    for (size_t i = 0; i < count; ++i) {
        if (source_count[i] && !output_count[i]) unsafe[i] |= 2;
        if (!source_count[i]) continue;
        if (diagnostics) {
            ++diagnostics[7];
            if (unsafe[i]) {
                ++diagnostics[8];
                diagnostics[9] += source_count[i];
                diagnostics[10] = std::max(diagnostics[10], source_count[i]);
                diagnostics[11] += (unsafe[i] & 1) != 0;
                diagnostics[12] += (unsafe[i] & 2) != 0;
            } else {
                diagnostics[13] += source_count[i];
                diagnostics[14] += output_count[i];
            }
        }
    }
    std::vector<unsigned int> restored;
    restored.reserve(before.size());
    for (size_t i = 0; i < after.size(); i += 3)
        if (!unsafe[parents[after[i]]])
            restored.insert(restored.end(), after.begin() + i, after.begin() + i + 3);
    for (size_t i = 0; i < before.size(); i += 3)
        if (unsafe[parents[before[i]]])
            restored.insert(restored.end(), before.begin() + i, before.begin() + i + 3);
    if (safe_topology(before, restored, count, original)) return false;
    after.swap(restored);
    return true;
}
}

extern "C" unsigned int horus_adaptive_mesh_abi() { return 1; }

static int simplify(
    const unsigned char* source, size_t vertex_count, size_t stride,
    const unsigned int* indices, size_t index_count,
    unsigned char* output_vertices, unsigned int* output_indices,
    size_t* counts, float ratio, float max_error, float color_weight,
    float* result_error, size_t* diagnostics, bool remove_degenerate) {
    if (!source || !indices || !output_vertices || !output_indices || !counts || !result_error ||
        !vertex_count || vertex_count > UINT32_MAX || index_count < 3 || index_count % 3 ||
        (stride != 12 && stride != 16) || !std::isfinite(ratio) || ratio <= 0 || ratio > 1 ||
        !std::isfinite(max_error) || max_error < 0 ||
        !std::isfinite(color_weight) || color_weight < 0) return -1;
    size_t local_diagnostics[16];
    if (!diagnostics) diagnostics = local_diagnostics;
    try {
        if (diagnostics) std::fill(diagnostics, diagnostics + 16, 0);
        for (size_t i = 0; i < index_count; ++i) if (indices[i] >= vertex_count) return -1;
        for (size_t i = 0; i < vertex_count; ++i) {
            float position[3];
            std::memcpy(position, source + i * stride, 12);
            for (float value : position) if (!std::isfinite(value)) return -1;
        }
        std::vector<unsigned int> remap(vertex_count);
        size_t welded_count = meshopt_generateVertexRemap(remap.data(), indices, index_count,
                                                         source, vertex_count, stride);
        std::vector<unsigned char> welded(welded_count * stride);
        std::vector<unsigned int> original(index_count);
        meshopt_remapVertexBuffer(welded.data(), source, vertex_count, stride, remap.data());
        meshopt_remapIndexBuffer(original.data(), indices, index_count, remap.data());
        std::vector<float> positions(welded_count * 3), attributes(welded_count * 4);
        for (size_t i = 0; i < welded_count; ++i) {
            std::memcpy(positions.data() + i * 3, welded.data() + i * stride, 12);
            if (stride == 16)
                for (size_t j = 0; j < 4; ++j)
                    attributes[i * 4 + j] = welded[i * stride + 12 + j] / 255.f;
        }
        std::vector<unsigned int> clean;
        clean.reserve(original.size());
        for (size_t i = 0; i < original.size(); i += 3) {
            if (zero_area(positions.data() + original[i] * 3,
                          positions.data() + original[i + 1] * 3,
                          positions.data() + original[i + 2] * 3)) continue;
            clean.insert(clean.end(), original.begin() + i, original.begin() + i + 3);
        }
        if (diagnostics) {
            diagnostics[0] = original.size() / 3;
            diagnostics[1] = clean.size() / 3;
            diagnostics[4] = (original.size() - clean.size()) / 3;
        }
        // A zero-area-only page remains representable by the existing HMSH upsert contract.
        bool degenerate_only = remove_degenerate && clean.empty();
        if (remove_degenerate && !degenerate_only) original.swap(clean);
        size_t working_count = original.size();
        Edges original_edges = edges(original);
        std::vector<unsigned char> locks(welded_count, 0);
        for (const auto& item : original_edges) {
            if (item.second == 2) continue;
            locks[item.first >> 32] = meshopt_SimplifyVertex_Lock;
            locks[unsigned(item.first)] = meshopt_SimplifyVertex_Lock;
        }
        size_t locked_count = std::count_if(locks.begin(), locks.end(),
                                           [](unsigned char value) { return value != 0; });
        std::vector<unsigned int> simplified(index_count);
        float weights[4] = {color_weight, color_weight, color_weight, color_weight};
        size_t target = std::max(size_t(3), size_t(working_count * double(ratio)) / 3 * 3);
        double extent = 0;
        for (size_t axis = 0; axis < 3; ++axis) {
            double low = positions[axis], high = low;
            for (size_t i = 0; i < welded_count; ++i) {
                low = std::min(low, double(positions[i * 3 + axis]));
                high = std::max(high, double(positions[i * 3 + axis]));
            }
            extent = std::max(extent, high - low);
        }
        size_t reduced;
        if (degenerate_only || extent > FLT_MAX || (extent > 0 && extent < 1.0 / FLT_MAX)) {
            // Finite float32 inputs can still overflow meshopt's normalization.
            simplified = original;
            reduced = working_count;
            *result_error = 0;
            if (diagnostics) diagnostics[3] = degenerate_only ? 8 : 7;
        } else {
            reduced = meshopt_simplifyWithAttributes(
                simplified.data(), original.data(), working_count, positions.data(), welded_count, 12,
                stride == 16 ? attributes.data() : nullptr, 16,
                stride == 16 ? weights : nullptr, stride == 16 ? 4 : 0, locks.data(),
                target, max_error, meshopt_SimplifyLockBorder | meshopt_SimplifyErrorAbsolute,
                result_error);
        }
        simplified.resize(reduced);
        unsigned int reason = !reduced ? 1 : reduced % 3 ? 2 :
            !std::isfinite(*result_error) ? 3 : *result_error > max_error ? 4 :
            safe_topology(original, simplified, welded_count, original_edges);
        if (diagnostics) {
            diagnostics[2] = reduced / 3;
            if (reason) diagnostics[3] = reason;
            uint32_t bits;
            std::memcpy(&bits, result_error, sizeof(bits));
            diagnostics[5] = bits;
            diagnostics[6] = original_edges.size();
        }
        if (reason) {
            bool recovered = remove_degenerate && (reason == 5 || reason == 6) &&
                restore_unsafe_components(original, simplified, welded_count, original_edges, diagnostics);
            if (!recovered) {
                simplified = original;
                *result_error = 0;
                if (diagnostics) diagnostics[15] = 1;
            } else if (diagnostics && diagnostics[13]) {
                diagnostics[3] = 9;
            } else {
                *result_error = 0;
            }
        }
        // Compact only the derived vertex buffer; every emitted attribute is original.
        std::fill(remap.begin(), remap.end(), UINT32_MAX);
        size_t output_count = 0;
        for (size_t i = 0; i < simplified.size(); ++i) {
            unsigned int index = simplified[i];
            if (remap[index] == UINT32_MAX) {
                remap[index] = unsigned(output_count++);
                std::memcpy(output_vertices + remap[index] * stride,
                            welded.data() + index * stride, stride);
            }
            output_indices[i] = remap[index];
        }
        counts[0] = output_count;
        counts[1] = simplified.size();
        counts[2] = welded_count;
        counts[3] = locked_count;
        return 0;
    } catch (const std::bad_alloc&) { return -2; }
      catch (...) { return -3; }
}

extern "C" int horus_adaptive_mesh(
    const unsigned char* source, size_t vertex_count, size_t stride,
    const unsigned int* indices, size_t index_count,
    unsigned char* output_vertices, unsigned int* output_indices,
    size_t* counts, float ratio, float max_error, float color_weight, float* result_error) {
    return simplify(source, vertex_count, stride, indices, index_count, output_vertices,
                    output_indices, counts, ratio, max_error, color_weight, result_error, nullptr, true);
}

extern "C" int horus_adaptive_mesh_diagnostics(
    const unsigned char* source, size_t vertex_count, size_t stride,
    const unsigned int* indices, size_t index_count,
    unsigned char* output_vertices, unsigned int* output_indices,
    size_t* counts, float ratio, float max_error, float color_weight,
    float* result_error, size_t* diagnostics) {
    return simplify(source, vertex_count, stride, indices, index_count, output_vertices,
                    output_indices, counts, ratio, max_error, color_weight, result_error, diagnostics, true);
}

extern "C" int horus_adaptive_mesh_baseline_diagnostics(
    const unsigned char* source, size_t vertex_count, size_t stride,
    const unsigned int* indices, size_t index_count,
    unsigned char* output_vertices, unsigned int* output_indices,
    size_t* counts, float ratio, float max_error, float color_weight,
    float* result_error, size_t* diagnostics) {
    return simplify(source, vertex_count, stride, indices, index_count, output_vertices,
                    output_indices, counts, ratio, max_error, color_weight, result_error, diagnostics, false);
}

extern "C" int horus_adaptive_mesh_restore_components(
    const unsigned int* source, size_t source_count,
    const unsigned int* candidate, size_t candidate_count, size_t vertex_count,
    unsigned int* output, size_t* output_count, size_t* diagnostics) {
    if (!source || !source_count || source_count % 3 || candidate_count % 3 ||
        (candidate_count && !candidate) || !output || !output_count || !diagnostics) return -1;
    try {
        std::fill(diagnostics, diagnostics + 16, 0);
        std::vector<unsigned int> before(source, source + source_count), after;
        if (candidate_count) after.assign(candidate, candidate + candidate_count);
        for (unsigned int index : before) if (index >= vertex_count) return -1;
        for (unsigned int index : after) if (index >= vertex_count) return -1;
        if (!restore_unsafe_components(before, after, vertex_count, edges(before), diagnostics)) return -4;
        std::copy(after.begin(), after.end(), output);
        *output_count = after.size();
        return 0;
    } catch (const std::bad_alloc&) { return -2; }
      catch (...) { return -3; }
}
