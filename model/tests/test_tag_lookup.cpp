#include <gtest/gtest.h>

#include <bit>
#include <iostream>
#include <random>

#include "apriltag.h"
#include "gv/model/tag_lookup.h"
#include "tag36h11.h"

using gv::model::Tag36h11Lookup;
using gv::model::TagMatch;

namespace {

const Tag36h11Lookup& lookup() {
  static const Tag36h11Lookup l;
  return l;
}

// Brute force over every rotated code: the nearest, first in table order on a tie.
int brute_nearest(uint64_t read, int* distance) {
  const auto& l = lookup();
  int best = -1, best_d = 65;
  for (int i = 0; i < l.num_rotated(); ++i) {
    const int d = std::popcount(read ^ l.rotated(i));
    if (d < best_d) best = i, best_d = d;
  }
  *distance = best_d;
  return best;
}

void expect_matches_brute(uint64_t read) {
  int want_d = 0;
  const int want = brute_nearest(read, &want_d);
  for (int max_bits = 0; max_bits <= 3; ++max_bits) {
    int d = -1;
    const int got = lookup().nearest(read, max_bits, &d);
    if (want_d > max_bits) {
      ASSERT_EQ(got, -1) << std::hex << read << " max_bits " << max_bits;
    } else {
      ASSERT_EQ(got, want) << std::hex << read << " max_bits " << max_bits;
      ASSERT_EQ(d, want_d);
    }
  }
}

uint64_t random_bits(std::mt19937_64& g, int count) {
  uint64_t m = 0;
  while (std::popcount(m) < count) m |= uint64_t{1} << (g() % 36);
  return m;
}

}  // namespace

TEST(TagLookup, FamilyShape) {
  const auto& l = lookup();
  EXPECT_EQ(l.num_codes(), 587);
  EXPECT_EQ(l.num_rotated(), 4 * 587);
  // Codes are at least 11 apart in every rotation, which is what makes a
  // match within 3 bits unique (and NAPA's early return correct).
  int least = 64;
  for (int i = 0; i < l.num_rotated(); ++i)
    for (int j = i + 1; j < l.num_rotated(); ++j) least = std::min(least, std::popcount(l.rotated(i) ^ l.rotated(j)));
  EXPECT_GE(least, 11);
}

TEST(TagLookup, FourRotationsAreIdentity) {
  for (int id = 0; id < lookup().num_codes(); ++id) {
    const uint64_t c = lookup().canonical(id);
    EXPECT_EQ(lookup().rotate(lookup().rotate(lookup().rotate(lookup().rotate(c)))), c);
  }
}

TEST(TagLookup, RotationMatchesRenderedTag) {
  // Read the bits back off AprilTag's own rendering turned 90 degrees CCW:
  // they must equal rotate() of the code.
  apriltag_family_t* fam = tag36h11_create();
  const auto& l = lookup();
  for (int id : {0, 1, 42, 100, 586}) {
    image_u8_t* im = apriltag_to_image(fam, id);
    const int n = im->width, off = (n - static_cast<int>(fam->width_at_border)) / 2;
    auto px = [&](int x, int y) { return im->buf[y * im->stride + x]; };
    uint64_t rotated_read = 0;
    for (int k = 0; k < 36; ++k) {
      // rotated image: out[y][x] = in[x][n-1-y]
      const int x = l.cell_x(k) + off, y = l.cell_y(k) + off;
      if (px(n - 1 - y, x)) rotated_read |= uint64_t{1} << (35 - k);
    }
    EXPECT_EQ(rotated_read, l.rotate(l.canonical(id))) << "id " << id;
    image_u8_destroy(im);
  }
  tag36h11_destroy(fam);
}

TEST(TagLookup, BucketsIndexEveryRotatedCodeOncePerBlock) {
  const auto& l = lookup();
  for (int b = 0; b < Tag36h11Lookup::kBlocks; ++b) {
    std::vector<int> seen(static_cast<size_t>(l.num_rotated()), 0);
    for (uint32_t v = 0; v < 512; ++v)
      for (uint32_t i : l.bucket(b, v)) {
        EXPECT_EQ((l.rotated(static_cast<int>(i)) >> (9 * b)) & 511u, v);
        seen[i]++;
      }
    for (int s : seen) EXPECT_EQ(s, 1);
  }
}

TEST(TagLookup, NearestMatchesBruteForce) {
  const auto& l = lookup();
  std::mt19937_64 g(1);
  // Every 0-2 bit error on a spread of codes, then random 3-6 bit errors and
  // random patterns.
  for (int i = 0; i < l.num_rotated(); i += 37) {
    const uint64_t c = l.rotated(i);
    expect_matches_brute(c);
    for (int a = 0; a < 36; ++a) {
      expect_matches_brute(c ^ (uint64_t{1} << a));
      for (int b = a + 1; b < 36; ++b) expect_matches_brute(c ^ (uint64_t{1} << a) ^ (uint64_t{1} << b));
    }
  }
  for (int n = 0; n < 20000; ++n) {
    const uint64_t c = l.rotated(static_cast<int>(g() % static_cast<uint64_t>(l.num_rotated())));
    expect_matches_brute(c ^ random_bits(g, 3 + n % 4));
    expect_matches_brute(g() & ((uint64_t{1} << 36) - 1));
  }
}

TEST(TagLookup, ConnectedThreeCellPlacementsMatchNapa) {
  // NAPA: "for tag36h11, only 420 of the 7,140 three-cell placements are connected".
  int connected = 0, total = 0;
  for (int a = 0; a < 36; ++a)
    for (int b = a + 1; b < 36; ++b)
      for (int c = b + 1; c < 36; ++c) {
        ++total;
        connected += lookup().errors_connected((uint64_t{1} << a) | (uint64_t{1} << b) | (uint64_t{1} << c));
      }
  EXPECT_EQ(total, 7140);
  EXPECT_EQ(connected, 420);
}

TEST(TagLookup, DecodeRules) {
  const auto& l = lookup();
  const int id = 123, rot = 2;
  const uint64_t c = l.rotated(rot * l.num_codes() + id);
  EXPECT_EQ(l.decode(c), (TagMatch{id, rot, 0}));
  // Find touching and scattered 3-bit errors by cell coordinates.
  auto bit_at = [&](int x, int y) {
    for (int k = 0; k < 36; ++k)
      if (l.cell_x(k) == x && l.cell_y(k) == y) return uint64_t{1} << (35 - k);
    return uint64_t{0};
  };
  const int bx = l.cell_x(0), by = l.cell_y(0);
  const uint64_t touching = bit_at(bx, by) | bit_at(bx + 1, by) | bit_at(bx + 1, by + 1);
  ASSERT_EQ(std::popcount(touching), 3);
  EXPECT_EQ(l.decode(c ^ touching), (TagMatch{id, rot, 3}));
  uint64_t scattered = 0;
  for (int k : {0, 17, 35}) scattered |= uint64_t{1} << (35 - k);
  ASSERT_FALSE(l.errors_connected(scattered));
  EXPECT_EQ(l.decode(c ^ scattered), std::nullopt);
  // Two scattered errors are always fine; four never decode.
  EXPECT_EQ(l.decode(c ^ (uint64_t{1} << 0) ^ (uint64_t{1} << 30)), (TagMatch{id, rot, 2}));
  uint64_t extra = 1;
  while (extra & touching) extra <<= 1;
  ASSERT_EQ(std::popcount(touching | extra), 4);
  EXPECT_EQ(l.decode(c ^ touching ^ extra), std::nullopt);
}

TEST(TagLookup, HardwareBudget) {
  // The candidate list the RTL compares per block lookup. Recorded in the
  // model README; this fails if a family change grows it unexpectedly.
  const size_t worst = lookup().max_bucket_size();
  RecordProperty("max_bucket_size", static_cast<int>(worst));
  std::cout << "[ budget ] max candidates per 9-bit bucket: " << worst
            << " (mean " << 4.0 * 587 / 512 << ")\n";
  EXPECT_LE(worst, 32u);
}
