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
float Scale() { return g_scale; }
ImVec4 WithAlpha(ImVec4 c, float a) {
  c.w = a;
  return c;
}

void ApplyTheme(bool dark, float scale) {
  g_dark = dark;
  g_scale = scale;
  Palette& p = g_pal;
  if (dark) {  // graphite, close to JetBrains / Unreal editor dark themes
    p.bg = Hex(0x1E1F22);
    p.panel = Hex(0x2B2D30);
    p.card = Hex(0x26282B);
    p.header = Hex(0x313337);
    p.card_border = Hex(0x3C3F44);
    p.field = Hex(0x1B1C1F);
    p.text = Hex(0xDFE1E5);
    p.text_dim = Hex(0x8E929B);
    p.accent = Hex(0x3574F0);
    p.accent_hover = Hex(0x4A85F5);
    p.accent_active = Hex(0x2A62D1);
    p.success = Hex(0x57A65F);
    p.warning = Hex(0xD9A343);
    p.danger = Hex(0xDB5C5C);
    p.plot_bg = Hex(0x1B1C1F);
    p.chrome = Hex(0x2B2D30);
  } else {  // light, close to classic engineering tools
    p.bg = Hex(0xEDEEF1);
    p.panel = Hex(0xF7F8FA);
    p.card = Hex(0xFFFFFF);
    p.header = Hex(0xF0F1F4);
    p.card_border = Hex(0xD3D5DB);
    p.field = Hex(0xFFFFFF);
    p.text = Hex(0x1F2023);
    p.text_dim = Hex(0x6C707E);
    p.accent = Hex(0x3574F0);
    p.accent_hover = Hex(0x4A85F5);
    p.accent_active = Hex(0x2A62D1);
    p.success = Hex(0x3F8F48);
    p.warning = Hex(0xB57A12);
    p.danger = Hex(0xC94343);
    p.plot_bg = Hex(0xFBFBFC);
    p.chrome = Hex(0xF7F8FA);
  }

  ImGuiStyle& s = ImGui::GetStyle();
  s = ImGuiStyle();
  ImVec4* c = s.Colors;
  c[ImGuiCol_Text] = p.text;
  c[ImGuiCol_TextDisabled] = p.text_dim;
  c[ImGuiCol_WindowBg] = p.bg;
  c[ImGuiCol_ChildBg] = ImVec4(0, 0, 0, 0);
  c[ImGuiCol_PopupBg] = p.panel;
  c[ImGuiCol_Border] = p.card_border;
  c[ImGuiCol_BorderShadow] = ImVec4(0, 0, 0, 0);
  c[ImGuiCol_FrameBg] = p.field;
  c[ImGuiCol_FrameBgHovered] = Mix(p.field, p.accent, 0.10f);
  c[ImGuiCol_FrameBgActive] = Mix(p.field, p.accent, 0.18f);
  c[ImGuiCol_TitleBg] = p.chrome;
  c[ImGuiCol_TitleBgActive] = p.chrome;
  c[ImGuiCol_MenuBarBg] = p.chrome;
  c[ImGuiCol_ScrollbarBg] = ImVec4(0, 0, 0, 0);
  c[ImGuiCol_ScrollbarGrab] = dark ? Hex(0x45484E) : Hex(0xC5C8CF);
  c[ImGuiCol_ScrollbarGrabHovered] = dark ? Hex(0x55585F) : Hex(0xAEB2BA);
  c[ImGuiCol_ScrollbarGrabActive] = p.accent;
  c[ImGuiCol_CheckMark] = p.accent;
  c[ImGuiCol_SliderGrab] = p.accent;
  c[ImGuiCol_SliderGrabActive] = p.accent_active;
  c[ImGuiCol_Button] = dark ? Hex(0x393B40) : Hex(0xFFFFFF);
  c[ImGuiCol_ButtonHovered] = dark ? Hex(0x43454B) : Hex(0xECEDF0);
  c[ImGuiCol_ButtonActive] = dark ? Hex(0x4E5157) : Hex(0xDFE1E5);
  c[ImGuiCol_Header] = WithAlpha(p.accent, dark ? 0.28f : 0.16f);
  c[ImGuiCol_HeaderHovered] = WithAlpha(p.accent, dark ? 0.18f : 0.10f);
  c[ImGuiCol_HeaderActive] = WithAlpha(p.accent, dark ? 0.36f : 0.22f);
  c[ImGuiCol_Separator] = p.card_border;
  c[ImGuiCol_SeparatorHovered] = p.accent;
  c[ImGuiCol_SeparatorActive] = p.accent;
  c[ImGuiCol_ResizeGrip] = ImVec4(0, 0, 0, 0);
  c[ImGuiCol_Tab] = p.panel;
  c[ImGuiCol_TabHovered] = p.header;
  c[ImGuiCol_TabSelected] = p.card;
  c[ImGuiCol_TabSelectedOverline] = p.accent;
  c[ImGuiCol_TableHeaderBg] = p.header;
  c[ImGuiCol_TableBorderStrong] = p.card_border;
  c[ImGuiCol_TableBorderLight] = Mix(p.card, p.card_border, 0.7f);
  c[ImGuiCol_TableRowBg] = ImVec4(0, 0, 0, 0);
  c[ImGuiCol_TableRowBgAlt] = dark ? ImVec4(1, 1, 1, 0.022f) : ImVec4(0, 0, 0, 0.022f);
  c[ImGuiCol_PlotLines] = p.accent;
  c[ImGuiCol_PlotHistogram] = p.accent;
  c[ImGuiCol_NavHighlight] = p.accent;
  c[ImGuiCol_TextSelectedBg] = WithAlpha(p.accent, 0.35f);
  c[ImGuiCol_ModalWindowDimBg] = ImVec4(0, 0, 0, 0.45f);

  s.WindowPadding = ImVec2(8, 8);
  s.FramePadding = ImVec2(7, 4);
  s.CellPadding = ImVec2(7, 4);
  s.ItemSpacing = ImVec2(7, 6);
  s.ItemInnerSpacing = ImVec2(6, 4);
  s.IndentSpacing = 16;
  s.ScrollbarSize = 10;
  s.GrabMinSize = 9;
  s.WindowBorderSize = 0;
  s.ChildBorderSize = 1;
  s.PopupBorderSize = 1;
  s.FrameBorderSize = 1;
  s.TabBorderSize = 0;
  s.WindowRounding = 3;
  s.ChildRounding = 2;
  s.FrameRounding = 2;
  s.PopupRounding = 3;
  s.ScrollbarRounding = 2;
  s.GrabRounding = 2;
  s.TabRounding = 2;
  s.SeparatorTextBorderSize = 1;
  s.SelectableTextAlign = ImVec2(0, 0.5f);
  s.ScaleAllSizes(scale);
}

void BeginCard(const char* icon, const char* title, const char* id) {
  const Palette& p = g_pal;
  const float fs = ImGui::GetFontSize();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, p.card);
  ImGui::PushStyleColor(ImGuiCol_Border, p.card_border);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12 * g_scale, 9 * g_scale));
  ImGui::BeginChild(id ? id : title, ImVec2(0, 0),
                    ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding);
  // Title bar across the full width of the section.
  const ImVec2 wp = ImGui::GetWindowPos();
  const float w = ImGui::GetWindowWidth();
  const float h = fs * 1.9f;
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(ImVec2(wp.x + 1, wp.y + 1), ImVec2(wp.x + w - 1, wp.y + h), ImGui::GetColorU32(p.header));
  dl->AddLine(ImVec2(wp.x + 1, wp.y + h), ImVec2(wp.x + w - 1, wp.y + h), ImGui::GetColorU32(p.card_border));
  const float ty = wp.y + (h - fs) * 0.5f;
  dl->AddText(ImVec2(wp.x + 12 * g_scale, ty), ImGui::GetColorU32(p.accent), icon);
  ImFont* bold = g_fonts.bold ? g_fonts.bold : ImGui::GetFont();
  dl->AddText(bold, fs, ImVec2(wp.x + 12 * g_scale + fs * 1.55f, ty), ImGui::GetColorU32(p.text), title);
  ImGui::SetCursorPosY(h + 9 * g_scale);
}

void EndCard() {
  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor(2);
  ImGui::Dummy(ImVec2(0, 3 * g_scale));
}

void PageHeader(const char* group, const char* icon, const char* title, const char* desc) {
  const Palette& p = g_pal;
  ImGui::PushStyleColor(ImGuiCol_Text, p.text_dim);
  ImGui::Text("%s  " ICON_FA_ANGLE_RIGHT "  %s", group, title);
  ImGui::PopStyleColor();
  if (g_fonts.title) ImGui::PushFont(g_fonts.title);
  ImGui::TextColored(p.accent, "%s", icon);
  ImGui::SameLine(0, ImGui::GetFontSize() * 0.5f);
  ImGui::TextUnformatted(title);
  if (g_fonts.title) ImGui::PopFont();
  ImGui::PushStyleColor(ImGuiCol_Text, p.text_dim);
  ImGui::TextWrapped("%s", desc);
  ImGui::PopStyleColor();
  ImGui::Dummy(ImVec2(0, 2 * g_scale));
  ImGui::Separator();
  ImGui::Dummy(ImVec2(0, 4 * g_scale));
}

void PanelTitle(const char* icon, const char* title) {
  const Palette& p = g_pal;
  const float fs = ImGui::GetFontSize();
  const ImVec2 wp = ImGui::GetWindowPos();
  const float w = ImGui::GetWindowWidth();
  const float h = fs * 1.9f;
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(wp, ImVec2(wp.x + w, wp.y + h), ImGui::GetColorU32(p.header));
  dl->AddLine(ImVec2(wp.x, wp.y + h - 1), ImVec2(wp.x + w, wp.y + h - 1), ImGui::GetColorU32(p.card_border));
  const float ty = wp.y + (h - fs) * 0.5f;
  dl->AddText(ImVec2(wp.x + fs * 0.7f, ty), ImGui::GetColorU32(p.text_dim), icon);
  ImFont* bold = g_fonts.bold ? g_fonts.bold : ImGui::GetFont();
  dl->AddText(bold, fs * 0.92f, ImVec2(wp.x + fs * 2.1f, ty + fs * 0.04f), ImGui::GetColorU32(p.text_dim), title);
  ImGui::SetCursorPosY(h + 4 * g_scale);
}

float LabelWidth() { return ImGui::GetFontSize() * 9.5f; }

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
  ImGui::PushStyleColor(ImGuiCol_Text, WithAlpha(g_pal.text_dim, 0.75f));
  ImGui::SetWindowFontScale(0.85f);
  ImGui::TextUnformatted(ICON_FA_CIRCLE_QUESTION);
  ImGui::SetWindowFontScale(1.0f);
  ImGui::PopStyleColor();
  if (ImGui::BeginItemTooltip()) {
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

void Splitter(const char* id, bool vertical, float length, float* size, float lo, float hi, bool invert) {
  const float t = 5.0f * g_scale;
  const ImVec2 pos = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(id, vertical ? ImVec2(t, length) : ImVec2(length, t));
  const bool hot = ImGui::IsItemHovered() || ImGui::IsItemActive();
  if (hot) ImGui::SetMouseCursor(vertical ? ImGuiMouseCursor_ResizeEW : ImGuiMouseCursor_ResizeNS);
  if (ImGui::IsItemActive()) {
    const float d = vertical ? ImGui::GetIO().MouseDelta.x : ImGui::GetIO().MouseDelta.y;
    *size = std::max(lo, std::min(hi, *size + (invert ? -d : d)));
  }
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImU32 col = ImGui::GetColorU32(hot ? g_pal.accent : g_pal.card_border);
  if (vertical)
    dl->AddLine(ImVec2(pos.x + t * 0.5f, pos.y), ImVec2(pos.x + t * 0.5f, pos.y + length), col, hot ? 2.0f : 1.0f);
  else
    dl->AddLine(ImVec2(pos.x, pos.y + t * 0.5f), ImVec2(pos.x + length, pos.y + t * 0.5f), col, hot ? 2.0f : 1.0f);
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
    push(ImGuiCol_ButtonHovered, Mix(base, ImVec4(1, 1, 1, 1), 0.12f));
    push(ImGuiCol_ButtonActive, Mix(base, ImVec4(0, 0, 0, 1), 0.12f));
    push(ImGuiCol_Border, Mix(base, ImVec4(0, 0, 0, 1), 0.15f));
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
  ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0, 0, 0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5 * g_scale, 3 * g_scale));
  bool r = ImGui::Button(icon);
  ImGui::PopStyleVar();
  ImGui::PopStyleColor(2);
  if (tooltip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tooltip);
  ImGui::PopID();
  return r;
}

bool ToolButton(const char* icon, const char* label, const ImVec4& icon_color, const char* tooltip, float height) {
  const Palette& p = g_pal;
  const float fs = ImGui::GetFontSize();
  const bool disabled = (ImGui::GetItemFlags() & ImGuiItemFlags_Disabled) != 0;
  const ImVec2 ts = ImGui::CalcTextSize(label);
  const float pad = fs * 0.6f;
  const ImVec2 size(pad * 2 + fs * 1.3f + ts.x, height);
  const ImVec2 pos = ImGui::GetCursorScreenPos();
  ImGui::PushID(label);
  const bool clicked = ImGui::InvisibleButton("##tb", size);
  const bool hov = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
  const bool act = ImGui::IsItemActive();
  ImGui::PopID();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  if ((hov || act) && !disabled) {
    dl->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y),
                      ImGui::GetColorU32(act ? ImGuiCol_ButtonActive : ImGuiCol_ButtonHovered), 3.0f * g_scale);
  }
  const float a = disabled ? 0.35f : 1.0f;
  const float ty = pos.y + (height - fs) * 0.5f;
  dl->AddText(ImVec2(pos.x + pad, ty), ImGui::GetColorU32(WithAlpha(icon_color, icon_color.w * a)), icon);
  dl->AddText(ImVec2(pos.x + pad + fs * 1.3f, ty), ImGui::GetColorU32(WithAlpha(p.text, a)), label);
  if (tooltip && hov && ImGui::BeginTooltip()) {
    ImGui::TextUnformatted(tooltip);
    ImGui::EndTooltip();
  }
  return clicked && !disabled;
}

void ToolSeparator(float height) {
  ImGui::SameLine(0, 6 * g_scale);
  const ImVec2 pos = ImGui::GetCursorScreenPos();
  ImGui::GetWindowDrawList()->AddLine(ImVec2(pos.x, pos.y + height * 0.2f), ImVec2(pos.x, pos.y + height * 0.8f),
                                      ImGui::GetColorU32(g_pal.card_border));
  ImGui::Dummy(ImVec2(1, height));
  ImGui::SameLine(0, 6 * g_scale);
}

void Pill(const char* text, const ImVec4& color) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImVec2 ts = ImGui::CalcTextSize(text);
  ImVec2 pad(7 * g_scale, 2 * g_scale);
  ImVec2 pos = ImGui::GetCursorScreenPos();
  float h = ImGui::GetFrameHeight();
  float y = pos.y + (h - ts.y - 2 * pad.y) * 0.5f;
  const ImVec2 a(pos.x, y), b(pos.x + ts.x + 2 * pad.x, y + ts.y + 2 * pad.y);
  dl->AddRectFilled(a, b, ImGui::GetColorU32(WithAlpha(color, 0.13f)), 3.0f * g_scale);
  dl->AddRect(a, b, ImGui::GetColorU32(WithAlpha(color, 0.45f)), 3.0f * g_scale);
  dl->AddText(ImVec2(pos.x + pad.x, y + pad.y), ImGui::GetColorU32(color), text);
  ImGui::Dummy(ImVec2(ts.x + 2 * pad.x, h));
}

void StatusDot(const ImVec4& color) {
  ImVec2 pos = ImGui::GetCursorScreenPos();
  float h = ImGui::GetTextLineHeight();
  float r = h * 0.2f;
  ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(pos.x + r + 1, pos.y + h * 0.5f + 1), r,
                                              ImGui::GetColorU32(color));
  ImGui::Dummy(ImVec2(r * 2 + 4, h));
}

void KpiTile(const char* label, const char* value, const char* unit, float width) {
  const Palette& p = g_pal;
  const float fs = ImGui::GetFontSize();
  ImFont* big = g_fonts.mono_big ? g_fonts.mono_big : ImGui::GetFont();
  const float h = fs * 0.95f + big->FontSize + fs * 0.9f;
  const ImVec2 pos = ImGui::GetCursorScreenPos();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(pos, ImVec2(pos.x + width, pos.y + h), ImGui::GetColorU32(p.field), 2.0f * g_scale);
  dl->AddRect(pos, ImVec2(pos.x + width, pos.y + h), ImGui::GetColorU32(p.card_border), 2.0f * g_scale);
  const float pad = fs * 0.55f;
  dl->AddText(ImGui::GetFont(), fs * 0.82f, ImVec2(pos.x + pad, pos.y + fs * 0.35f), ImGui::GetColorU32(p.text_dim), label);
  // Value right-aligned like a panel meter, unit after it.
  const float us = fs * 0.82f;
  const float uw = unit && *unit ? ImGui::GetFont()->CalcTextSizeA(us, 1e9f, 0, unit).x + fs * 0.3f : 0.0f;
  const ImVec2 vs = big->CalcTextSizeA(big->FontSize, 1e9f, 0, value);
  const float vy = pos.y + fs * 1.2f;
  dl->AddText(big, big->FontSize, ImVec2(pos.x + width - pad - uw - vs.x, vy), ImGui::GetColorU32(p.text), value);
  if (uw > 0)
    dl->AddText(ImGui::GetFont(), us, ImVec2(pos.x + width - pad - uw + fs * 0.3f, vy + big->FontSize - us - fs * 0.1f),
                ImGui::GetColorU32(p.text_dim), unit);
  ImGui::Dummy(ImVec2(width, h));
}

bool FoldHeader(const char* icon, const char* title, bool default_open, bool force_open) {
  const Palette& p = g_pal;
  const float fs = ImGui::GetFontSize();
  ImGuiStorage* st = ImGui::GetStateStorage();
  const ImGuiID id = ImGui::GetID(title);
  bool open = force_open || st->GetBool(id, default_open);
  const ImVec2 pos = ImGui::GetCursorScreenPos();
  const float w = ImGui::GetContentRegionAvail().x, h = fs * 1.75f;
  if (ImGui::InvisibleButton(title, ImVec2(w, h)) && !force_open) {
    open = !open;
    st->SetBool(id, open);
  }
  const bool hov = ImGui::IsItemHovered();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(pos, ImVec2(pos.x + w, pos.y + h), ImGui::GetColorU32(hov ? Mix(p.header, p.accent, 0.10f) : p.header), 2.0f);
  dl->AddRect(pos, ImVec2(pos.x + w, pos.y + h), ImGui::GetColorU32(p.card_border), 2.0f);
  const float ty = pos.y + (h - fs) * 0.5f;
  dl->AddText(ImGui::GetFont(), fs * 0.72f, ImVec2(pos.x + fs * 0.55f, ty + fs * 0.14f), ImGui::GetColorU32(p.text_dim),
              open ? ICON_FA_CHEVRON_DOWN : ICON_FA_CHEVRON_RIGHT);
  dl->AddText(ImVec2(pos.x + fs * 1.5f, ty), ImGui::GetColorU32(p.accent), icon);
  ImFont* bold = g_fonts.bold ? g_fonts.bold : ImGui::GetFont();
  dl->AddText(bold, fs, ImVec2(pos.x + fs * 3.0f, ty), ImGui::GetColorU32(p.text), title);
  if (open) ImGui::Dummy(ImVec2(0, 1));
  return open;
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
