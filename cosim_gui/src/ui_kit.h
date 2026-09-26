// Small design system on top of Dear ImGui, in the style of desktop
// engineering tools: neutral graphite theme, flat section panels with title
// bars, property-grid rows, toolbar buttons, splitters and instrument
// readouts. Keeps every panel visually consistent.
#pragma once

#include <string>

#include "IconsFontAwesome6.h"
#include "imgui.h"

namespace ui {

struct Fonts {
  ImFont* regular = nullptr;   // CJK + icons, body text
  ImFont* bold = nullptr;      // CJK bold + icons, section titles
  ImFont* title = nullptr;     // CJK bold, larger: page titles
  ImFont* mono = nullptr;      // monospace digits: readouts, log times
  ImFont* mono_big = nullptr;  // large monospace digits: instrument values
};
Fonts& GetFonts();

struct Palette {
  ImVec4 bg;           // workspace background
  ImVec4 panel;        // docked panels (tree, monitor, console)
  ImVec4 card;         // section body
  ImVec4 header;       // section / panel title bars
  ImVec4 card_border;  // 1 px borders
  ImVec4 field;        // input fields, readout insets
  ImVec4 text, text_dim;
  ImVec4 accent, accent_hover, accent_active;
  ImVec4 success, warning, danger;
  ImVec4 plot_bg;
  ImVec4 chrome;       // menu bar, toolbar, status bar
};
const Palette& Colors();

void ApplyTheme(bool dark, float scale);
bool IsDark();
float Scale();
ImVec4 WithAlpha(ImVec4 c, float a);

// ---- layout -------------------------------------------------------------
// Section panel with a title bar; always pair with EndCard(). Auto-sizes to
// its content height.
void BeginCard(const char* icon, const char* title, const char* id = nullptr);
void EndCard();

// Page heading of the workspace: breadcrumb, title and one-line description.
void PageHeader(const char* group, const char* icon, const char* title, const char* desc);

// Title bar of a docked panel (tree, monitor, console).
void PanelTitle(const char* icon, const char* title);

// Aligned property row: label in a fixed left column, widget fills the rest.
// Widgets after Row() should use a hidden label ("##id").
void Row(const char* label, const char* help = nullptr, float widget_width = 0.0f);
float LabelWidth();

void HelpMarker(const char* text);

// Draggable divider between two panes. vertical = a vertical bar that moves
// in x. *size is the pane size it controls; invert when the pane is on the
// far side (right / bottom) of the bar.
void Splitter(const char* id, bool vertical, float length, float* size, float lo, float hi, bool invert);

// ---- widgets --------------------------------------------------------------
enum class Kind { Primary, Secondary, Danger, Success };
bool Button(const char* icon, const char* text, Kind kind = Kind::Secondary, ImVec2 size = ImVec2(0, 0));
// Secondary text that wraps at the panel edge instead of being cut off.
void DimWrapped(const char* fmt, ...) IM_FMTARGS(1);
bool IconButton(const char* icon, const char* tooltip, const char* id);
// Flat toolbar button: coloured icon + label, frame only on hover.
bool ToolButton(const char* icon, const char* label, const ImVec4& icon_color, const char* tooltip, float height);
void ToolSeparator(float height);
void Pill(const char* text, const ImVec4& color);
void StatusDot(const ImVec4& color);
// Instrument readout: small caption, large monospace value, unit.
void KpiTile(const char* label, const char* value, const char* unit, float width);
void SectionCaption(const char* text);
// Flat fold-out header (monitor sections, optional groups). Returns open.
// Click targets for the --tour self-test: widgets record where they are so
// the tour can press them with real mouse events.
void RecordTarget(const std::string& name);
// The target the scripted tour is about to click: RecordTarget scrolls it into view.
void SetTourTarget(const std::string& name);
bool FindTarget(const std::string& name, ImVec2* center);

bool FoldHeader(const char* icon, const char* title, bool default_open = true, bool force_open = false);

}  // namespace ui
