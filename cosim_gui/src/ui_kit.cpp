#include "ui_kit.h"

#include <algorithm>

#include "imgui_internal.h"

namespace ui {

namespace {
Fonts g_fonts;
Palette g_pal;
bool g_dark = true;
float g_scale = 1.0f;

ImVec4 Hex(unsigned rgb, float a = 1.0f) {
  return ImVec4(((rgb >> 16) & 0xFF) / 255.0f, ((rgb >> 8) & 0xFF) / 255.0f, (rgb & 0xFF) / 255.0f, a);
}
ImVec4 Mix(const ImVec4& a, const ImVec4& b, float t) {
  return ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
}
}  // namespace

Fonts& GetFonts() { return g_fonts; }
const Palette& Colors() { return g_pal; }
bool IsDark() { return g_dark; }

void ApplyTheme(bool dark, float scale) {
  g_dark = dark;
  g_scale = scale;
  ImGuiStyle& s = ImGui::GetStyle();
  s = ImGuiStyle();
  if (dark) {
    g_pal = {Hex(0x14161A), Hex(0x1B1E24), Hex(0x21252C), Hex(0x2E333C), Hex(0xE6E8EB), Hex(0x8B93A1),
             Hex(0x3B82F6), Hex(0x5B9BFF), Hex(0x2F6FD8), Hex(0x22C55E), Hex(0xF59E0B), Hex(0xEF4444),
             Hex(0x181B20)};
  } else {  // light, close to classic engineering tools such as CarSim
    g_pal = {Hex(0xE9ECF1), Hex(0xF4F6F9), Hex(0xFFFFFF), Hex(0xD3D8E0), Hex(0x1F2733), Hex(0x6B7483),
             Hex(0x1F6FEB), Hex(0x3D86F5), Hex(0x185CC4), Hex(0x16A34A), Hex(0xD97706), Hex(0xDC2626),
             Hex(0xFAFBFC)};
  }
  const Palette& p = g_pal;
  ImVec4* c = s.Colors;
  c[ImGuiCol_Text] = p.text;
  c[ImGuiCol_TextDisabled] = p.text_dim;
  c[ImGuiCol_WindowBg] = p.bg;
  c[ImGuiCol_ChildBg] = p.panel;
  c[ImGuiCol_PopupBg] = p.card;
  c[ImGuiCol_Border] = p.card_border;
  c[ImGuiCol_BorderShadow] = ImVec4(0, 0, 0, 0);
  const ImVec4 field = dark ? Hex(0x2A2F38) : Hex(0xF0F2F5);
  c[ImGuiCol_FrameBg] = field;
  c[ImGuiCol_FrameBgHovered] = Mix(field, p.accent, 0.18f);
  c[ImGuiCol_FrameBgActive] = Mix(field, p.accent, 0.30f);
  c[ImGuiCol_TitleBg] = p.panel;
  c[ImGuiCol_TitleBgActive] = p.panel;
  c[ImGuiCol_MenuBarBg] = dark ? Hex(0x101216) : Hex(0xDDE2E9);
  c[ImGuiCol_ScrollbarBg] = ImVec4(0, 0, 0, 0);
  c[ImGuiCol_ScrollbarGrab] = dark ? Hex(0x3A404B) : Hex(0xC3CAD4);
  c[ImGuiCol_ScrollbarGrabHovered] = dark ? Hex(0x4A5160) : Hex(0xAAB3BF);
  c[ImGuiCol_ScrollbarGrabActive] = p.accent;
  c[ImGuiCol_CheckMark] = p.accent;
  c[ImGuiCol_SliderGrab] = p.accent;
  c[ImGuiCol_SliderGrabActive] = p.accent_active;
  c[ImGuiCol_Button] = dark ? Hex(0x2C323C) : Hex(0xE4E8EE);
  c[ImGuiCol_ButtonHovered] = dark ? Hex(0x363D49) : Hex(0xD6DCE4);
  c[ImGuiCol_ButtonActive] = dark ? Hex(0x404857) : Hex(0xC8CFD9);
  c[ImGuiCol_Header] = Mix(p.panel, p.accent, 0.22f);
  c[ImGuiCol_HeaderHovered] = Mix(p.panel, p.accent, 0.32f);
  c[ImGuiCol_HeaderActive] = Mix(p.panel, p.accent, 0.45f);
  c[ImGuiCol_Separator] = p.card_border;
  c[ImGuiCol_Tab] = p.panel;
  c[ImGuiCol_TabHovered] = Mix(p.panel, p.accent, 0.35f);
  c[ImGuiCol_TabSelected] = p.card;
  c[ImGuiCol_TabSelectedOverline] = p.accent;
  c[ImGuiCol_TableHeaderBg] = dark ? Hex(0x262B33) : Hex(0xEEF1F5);
  c[ImGuiCol_TableBorderStrong] = p.card_border;
  c[ImGuiCol_TableBorderLight] = Mix(p.card, p.card_border, 0.6f);
  c[ImGuiCol_TableRowBg] = ImVec4(0, 0, 0, 0);
  c[ImGuiCol_TableRowBgAlt] = dark ? ImVec4(1, 1, 1, 0.025f) : ImVec4(0, 0, 0, 0.025f);
  c[ImGuiCol_PlotLines] = p.accent;
  c[ImGuiCol_PlotHistogram] = p.accent;
  c[ImGuiCol_NavHighlight] = p.accent;
  c[ImGuiCol_TextSelectedBg] = Mix(p.card, p.accent, 0.4f);

  s.WindowPadding = ImVec2(10, 10);
  s.FramePadding = ImVec2(9, 5);
  s.CellPadding = ImVec2(8, 5);
  s.ItemSpacing = ImVec2(9, 7);
  s.ItemInnerSpacing = ImVec2(7, 5);
  s.ScrollbarSize = 11;
  s.GrabMinSize = 10;
  s.WindowBorderSize = 0;
  s.ChildBorderSize = 1;
  s.FrameBorderSize = 0;
  s.WindowRounding = 0;
  s.ChildRounding = 7;
  s.FrameRounding = 5;
  s.PopupRounding = 6;
  s.ScrollbarRounding = 6;
  s.GrabRounding = 5;
  s.TabRounding = 5;
  s.SeparatorTextBorderSize = 1;
  s.ScaleAllSizes(scale);
}

void BeginCard(const char* icon, const char* title, const char* id) {
  const Palette& p = g_pal;
  ImGui::PushStyleColor(ImGuiCol_ChildBg, p.card);
  ImGui::PushStyleColor(ImGuiCol_Border, p.card_border);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14 * g_scale, 11 * g_scale));
  ImGui::BeginChild(id ? id : title, ImVec2(0, 0),
                    ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding);
  if (g_fonts.bold) ImGui::PushFont(g_fonts.bold);
  ImGui::PushStyleColor(ImGuiCol_Text, p.accent);
  ImGui::TextUnformatted(icon);
  ImGui::PopStyleColor();
  ImGui::SameLine();
  ImGui::TextUnformatted(title);
  if (g_fonts.bold) ImGui::PopFont();
  ImGui::Dummy(ImVec2(0, 2 * g_scale));
}

void EndCard() {
  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor(2);
  ImGui::Dummy(ImVec2(0, 4 * g_scale));
}

float LabelWidth() { return ImGui::GetFontSize() * 8.5f; }

void Row(const char* label, const char* help, float widget_width) {
  ImGui::AlignTextToFramePadding();
  ImGui::PushStyleColor(ImGuiCol_Text, g_pal.text_dim);
  ImGui::TextUnformatted(label);
  ImGui::PopStyleColor();
  if (help) {
    ImGui::SameLine(0, 4);
    HelpMarker(help);
  }
  ImGui::SameLine(LabelWidth());
  float avail = ImGui::GetContentRegionAvail().x;
  ImGui::SetNextItemWidth(widget_width > 0 ? std::min(widget_width, avail) : std::min(avail, ImGui::GetFontSize() * 26));
}

void HelpMarker(const char* text) {
  ImGui::PushStyleColor(ImGuiCol_Text, g_pal.text_dim);
  ImGui::TextUnformatted(ICON_FA_CIRCLE_QUESTION);
  ImGui::PopStyleColor();
  if (ImGui::BeginItemTooltip()) {
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

bool Button(const char* icon, const char* text, Kind kind, ImVec2 size) {
  const Palette& p = g_pal;
  int pushed = 0;
  auto push = [&](ImGuiCol idx, ImVec4 col) { ImGui::PushStyleColor(idx, col); ++pushed; };
  ImVec4 base;
  switch (kind) {
    case Kind::Primary: base = p.accent; break;
    case Kind::Danger: base = p.danger; break;
    case Kind::Success: base = p.success; break;
    default: base = ImVec4(0, 0, 0, 0); break;
  }
  if (kind != Kind::Secondary) {
    push(ImGuiCol_Button, base);
    push(ImGuiCol_ButtonHovered, Mix(base, ImVec4(1, 1, 1, 1), 0.15f));
    push(ImGuiCol_ButtonActive, Mix(base, ImVec4(0, 0, 0, 1), 0.15f));
    push(ImGuiCol_Text, ImVec4(1, 1, 1, 1));
  }
  std::string label = icon && *icon ? std::string(icon) + "  " + text : std::string(text);
  bool r = ImGui::Button(label.c_str(), size);
  ImGui::PopStyleColor(pushed);
  return r;
}

bool IconButton(const char* icon, const char* tooltip, const char* id) {
  ImGui::PushID(id);
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5 * g_scale, 3 * g_scale));
  bool r = ImGui::Button(icon);
  ImGui::PopStyleVar();
  ImGui::PopStyleColor();
  if (tooltip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tooltip);
  ImGui::PopID();
  return r;
}

void Pill(const char* text, const ImVec4& color) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImVec2 ts = ImGui::CalcTextSize(text);
  ImVec2 pad(9 * g_scale, 3 * g_scale);
  ImVec2 pos = ImGui::GetCursorScreenPos();
  float h = ImGui::GetFrameHeight();
  float y = pos.y + (h - ts.y - 2 * pad.y) * 0.5f;
  ImVec4 bg = color;
  bg.w = 0.18f;
  dl->AddRectFilled(ImVec2(pos.x, y), ImVec2(pos.x + ts.x + 2 * pad.x, y + ts.y + 2 * pad.y),
                    ImGui::GetColorU32(bg), 20.0f);
  dl->AddText(ImVec2(pos.x + pad.x, y + pad.y), ImGui::GetColorU32(color), text);
  ImGui::Dummy(ImVec2(ts.x + 2 * pad.x, h));
}

void StatusDot(const ImVec4& color) {
  ImVec2 pos = ImGui::GetCursorScreenPos();
  float h = ImGui::GetTextLineHeight();
  float r = h * 0.22f;
  ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(pos.x + r + 1, pos.y + h * 0.5f + 1), r,
                                              ImGui::GetColorU32(color));
  ImGui::Dummy(ImVec2(r * 2 + 4, h));
}

void KpiTile(const char* label, const char* value, const char* unit, float width) {
  const Palette& p = g_pal;
  ImGui::PushStyleColor(ImGuiCol_ChildBg, g_dark ? Mix(p.card, p.accent, 0.06f) : Mix(p.card, p.accent, 0.04f));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10 * g_scale, 7 * g_scale));
  ImGui::BeginChild(label, ImVec2(width, 0), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY |
                                                 ImGuiChildFlags_AlwaysUseWindowPadding);
  ImGui::PushStyleColor(ImGuiCol_Text, p.text_dim);
  ImGui::TextUnformatted(label);
  ImGui::PopStyleColor();
  if (g_fonts.big) ImGui::PushFont(g_fonts.big);
  ImGui::TextUnformatted(value);
  if (g_fonts.big) ImGui::PopFont();
  if (unit && *unit) {
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Text, p.text_dim);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(unit);
    ImGui::PopStyleColor();
  }
  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor();
}

void SectionCaption(const char* text) {
  ImGui::Dummy(ImVec2(0, 4 * g_scale));
  ImGui::PushStyleColor(ImGuiCol_Text, g_pal.text_dim);
  ImGui::SetWindowFontScale(0.86f);
  ImGui::TextUnformatted(text);
  ImGui::SetWindowFontScale(1.0f);
  ImGui::PopStyleColor();
}

}  // namespace ui
