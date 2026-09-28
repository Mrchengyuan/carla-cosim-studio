#pragma once
// The main camera's preset viewpoints (views.py mode_mount): the six
// directions, then the two close-ups. The toolbar over the viewport, the
// 实时画面 page and a pane's source list all use this table.
#include "IconsFontAwesome6.h"

struct ViewModeDef {
  const char* id;    // views.py mode
  const char* icon;
  const char* text;  // toolbar label
  const char* name;  // longer name (page, pane source)
  const char* tip;
};

inline constexpr ViewModeDef kViewModes[] = {
    {"chase", ICON_FA_CAR_REAR, "后", "后方（跟车）", "从车后上方看（跟车）"},
    {"front", ICON_FA_CAR, "前", "前方", "从车的前方往回看车头"},
    {"left", ICON_FA_ARROW_LEFT, "左", "左侧", "从车的左侧看"},
    {"right", ICON_FA_ARROW_RIGHT, "右", "右侧", "从车的右侧看"},
    {"top", ICON_FA_ARROWS_TO_EYE, "俯视", "俯视", "从正上方往下看"},
    {"iso", ICON_FA_CUBE, "斜上", "斜上方", "从左后方的斜上方看"},
    {"hood", ICON_FA_EYE, "车头", "车头", "从车头往前看（驾驶员视角）"},
    {"wheel", ICON_FA_CIRCLE_DOT, "前轮", "前轮特写", "看转向、车轮转动和悬架跳动"},
};
inline constexpr int kViewModeCount = static_cast<int>(sizeof(kViewModes) / sizeof(kViewModes[0]));
