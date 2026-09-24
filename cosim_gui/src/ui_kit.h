// Small design system on top of Dear ImGui: themes, cards, aligned form
// rows, icon buttons and status pills. Keeps every panel visually consistent.
#pragma once

#include <string>

#include "IconsFontAwesome6.h"
#include "imgui.h"

namespace ui {

struct Fonts {
  ImFont* regular = nullptr;  // CJK + icons, body text
  ImFont* bold = nullptr;     // CJK bold + icons, card titles / nav
  ImFont* big = nullptr;      // large digits for KPI tiles
};
Fonts& GetFonts();

struct Palette {
  ImVec4 bg, panel, card, card_border, text, text_dim, accent, accent_hover, accent_active,
      success, warning, danger, plot_bg;
};
const Palette& Colors();

void ApplyTheme(bool dark, float scale);
bool IsDark();

// ---- layout -------------------------------------------------------------
// Titled card; always pair with EndCard(). Auto-sizes to its content height.
void BeginCard(const char* icon, const char* title, const char* id = nullptr);
void EndCard();

// Aligned form row: label in a fixed left column, widget fills the rest.
// Widgets after Row() should use a hidden label ("##id").
void Row(const char* label, const char* help = nullptr, float widget_width = 0.0f);
float LabelWidth();

void HelpMarker(const char* text);

// ---- widgets --------------------------------------------------------------
enum class Kind { Primary, Secondary, Danger, Success };
bool Button(const char* icon, const char* text, Kind kind = Kind::Secondary, ImVec2 size = ImVec2(0, 0));
bool IconButton(const char* icon, const char* tooltip, const char* id);
void Pill(const char* text, const ImVec4& color);
void StatusDot(const ImVec4& color);
void KpiTile(const char* label, const char* value, const char* unit, float width);
void SectionCaption(const char* text);

}  // namespace ui
