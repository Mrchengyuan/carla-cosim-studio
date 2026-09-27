// JPEG live view frames from a backend on a server (views.py sends them when
// a remote GUI's hello asks), decoded with the vendored stb_image
// (third_party/stb, JPEG only).
#pragma once

#include <vector>

// jpg -> rgb (w x h x 3 bytes). False, with rgb, w and h unchanged, when stb_image cannot read it.
bool DecodeJpeg(const std::vector<unsigned char>& jpg, std::vector<unsigned char>& rgb, int& w, int& h);
