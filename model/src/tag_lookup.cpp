#include "gv/model/tag_lookup.h"

#include <algorithm>
#include <bit>
#include <stdexcept>

#include "apriltag.h"
#include "tag36h11.h"

namespace gv::model {

namespace {

// Bit k is stored MSB first: bit 0 of the cell order is bit 35 of the word.
constexpr uint64_t bit_mask(int k) { return uint64_t{1} << (Tag36h11Lookup::kBits - 1 - k); }

}  // namespace

Tag36h11Lookup::Tag36h11Lookup() {
  apriltag_family_t* fam = tag36h11_create();
  if (fam->nbits != static_cast<uint32_t>(kBits)) throw std::logic_error("tag36h11 must have 36 bits");
  canonical_.assign(fam->codes, fam->codes + fam->ncodes);
  int lo = 1 << 30, hi = -(1 << 30);
  for (int k = 0; k < kBits; ++k) {
    cell_x_[static_cast<size_t>(k)] = static_cast<int>(fam->bit_x[k]);
    cell_y_[static_cast<size_t>(k)] = static_cast<int>(fam->bit_y[k]);
    lo = std::min({lo, cell_x_[static_cast<size_t>(k)], cell_y_[static_cast<size_t>(k)]});
    hi = std::max({hi, cell_x_[static_cast<size_t>(k)], cell_y_[static_cast<size_t>(k)]});
  }
  tag36h11_destroy(fam);

  // Rotation as a bit permutation, from cell coordinates on the data grid:
  // out[y][x] = in[x][n-1-y], with grid coordinates relative to `lo`.
  const int n = hi - lo + 1;
  for (int k = 0; k < kBits; ++k) {
    const int gx = cell_x_[static_cast<size_t>(k)] - lo, gy = cell_y_[static_cast<size_t>(k)] - lo;
    const int sx = n - 1 - gy, sy = gx;
    int src = -1;
    for (int j = 0; j < kBits; ++j)
      if (cell_x_[static_cast<size_t>(j)] - lo == sx && cell_y_[static_cast<size_t>(j)] - lo == sy) src = j;
    if (src < 0) throw std::logic_error("tag36h11 layout doesn't rotate onto itself");
    rot_src_[static_cast<size_t>(k)] = src;
    for (int j = 0; j < kBits; ++j) {
      const int dx = cell_x_[static_cast<size_t>(j)] - cell_x_[static_cast<size_t>(k)];
      const int dy = cell_y_[static_cast<size_t>(j)] - cell_y_[static_cast<size_t>(k)];
      if (j != k && dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1) near_[static_cast<size_t>(k)] |= bit_mask(j);
    }
  }

  const auto ncodes = canonical_.size();
  rotated_.resize(4 * ncodes);
  for (size_t id = 0; id < ncodes; ++id) {
    uint64_t c = canonical_[id];
    for (size_t r = 0; r < 4; ++r) {
      rotated_[r * ncodes + id] = c;
      c = rotate(c);
    }
  }

  for (int b = 0; b < kBlocks; ++b) {
    auto& table = buckets_[static_cast<size_t>(b)];
    table.assign(size_t{1} << kBlockBits, {});
    for (size_t i = 0; i < rotated_.size(); ++i)
      table[(rotated_[i] >> (b * kBlockBits)) & ((1u << kBlockBits) - 1)].push_back(static_cast<uint32_t>(i));
  }
}

uint64_t Tag36h11Lookup::rotate(uint64_t code) const {
  uint64_t out = 0;
  for (int k = 0; k < kBits; ++k)
    if (code & bit_mask(rot_src_[static_cast<size_t>(k)])) out |= bit_mask(k);
  return out;
}

int Tag36h11Lookup::nearest(uint64_t read, int max_bits, int* distance) const {
  if (max_bits < 0 || max_bits > 3)
    throw std::invalid_argument("nearest: four exact blocks only cover up to 3 wrong bits");
  // With codes >= 11 apart, a candidate within 3 bits is the only one, so the
  // first hit is the answer (NAPA returns on d <= max_hamming && d <= max_bits).
  for (int b = 0; b < kBlocks; ++b) {
    const auto v = static_cast<uint32_t>((read >> (b * kBlockBits)) & ((1u << kBlockBits) - 1));
    for (uint32_t i : bucket(b, v)) {
      const int d = std::popcount(read ^ rotated_[i]);
      if (d <= max_bits) {
        if (distance) *distance = d;
        return static_cast<int>(i);
      }
    }
  }
  return -1;
}

std::optional<TagMatch> Tag36h11Lookup::decode(uint64_t read, int max_bits) const {
  int d = 0;
  const int i = nearest(read, max_bits, &d);
  if (i < 0) return std::nullopt;
  if (d == 3 && !errors_connected(read ^ rotated_[static_cast<size_t>(i)])) return std::nullopt;
  const int nc = num_codes();
  return TagMatch{i % nc, i / nc, d};
}

bool Tag36h11Lookup::errors_connected(uint64_t diff) const {
  if (std::popcount(diff) <= 1) return true;
  uint64_t reached = diff & (~diff + 1), last = 0;  // start from the lowest set bit
  while (reached != last) {
    last = reached;
    for (int k = 0; k < kBits; ++k)
      if (reached & bit_mask(k)) reached |= near_[static_cast<size_t>(k)] & diff;
  }
  return reached == diff;
}

size_t Tag36h11Lookup::max_bucket_size() const {
  size_t m = 0;
  for (const auto& table : buckets_)
    for (const auto& list : table) m = std::max(m, list.size());
  return m;
}

}  // namespace gv::model
