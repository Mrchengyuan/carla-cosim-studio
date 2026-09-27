// Fonts of the GUI and the remote launcher (shared).
#pragma once

#include <string>

namespace ui {
// CJK text font + Font Awesome icons merged in; bold variants for section and
// page titles; a monospace font for instrument readouts. font_override: a CJK
// font file to use instead of the system's (--font).
void LoadFonts(float scale, const std::string& font_override);
}  // namespace ui
