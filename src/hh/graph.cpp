// PID graph: draws the latest chassis motion on the Brain screen.
//
// The chassis calls begin() when a motion starts and record() every 10 ms
// during it. A separate task (graph_task) draws what has been recorded, so
// drawing never slows the motion down.

#include "vex.h"

namespace hh {
namespace graph {

bool enabled = false;

// One point per pixel column at most. When the buffer fills, every other
// point is dropped and recording carries on at half the rate, so a motion of
// any length fits.
static const int MAX_POINTS = 230;
static float errors[MAX_POINTS];
static float outputs[MAX_POINTS];
static int count = 0;
static int stride = 1;
static int skipped = 0;

// label / units        what the motion is ("Drive", "in")
// recording            a motion is sending points right now
// showing              the graph has the Brain screen
// dirty                there is something new to draw
static const char* label = "";
static const char* units = "";
static uint32_t begin_ms = 0;
static uint32_t last_record_ms = 0;
static bool recording = false;
static bool showing = false;
static bool dirty = false;

static float first_error = 0;
static float last_error = 0;
static float worst_overshoot = 0;

// A motion is starting: throw away the old graph and take the screen.
void begin(const char* motion_label, const char* motion_units) {
  if (!enabled) return;
  label = motion_label;
  units = motion_units;
  count = 0;
  stride = 1;
  skipped = 0;
  first_error = 0;
  last_error = 0;
  worst_overshoot = 0;
  begin_ms = now_ms();
  last_record_ms = begin_ms;
  recording = true;
  showing = true;
  dirty = true;
}

// One point of the motion: how far from the target, and the voltage sent.
void record(float error, float output) {
  if (!enabled || !recording) return;
  last_record_ms = now_ms();
  if (count == 0 && skipped == 0) first_error = error;
  last_error = error;
  // Past the target means the error has the opposite sign to how it started.
  float past = first_error >= 0 ? -error : error;
  if (past > worst_overshoot) worst_overshoot = past;

  if (++skipped < stride) return;
  skipped = 0;
  if (count == MAX_POINTS) {
    for (int i = 0; i < MAX_POINTS / 2; i++) {
      errors[i] = errors[i * 2];
      outputs[i] = outputs[i * 2];
    }
    count = MAX_POINTS / 2;
    stride *= 2;
  }
  errors[count] = error;
  outputs[count] = output;
  count++;
  dirty = true;
}

// Numbers about the latest motion, for the screen and for teams to read.
float duration_ms() { return last_record_ms - begin_ms; }
float start_error() { return first_error; }
float final_error() { return last_error; }
float overshoot() { return worst_overshoot; }
int samples() { return count; }
bool owns_screen() { return enabled && showing; }

static const int LEFT = 10;
static const int WIDTH = 460;

// The screen x position for point number `index` out of `total`.
static int column(int index, int total) {
  if (total <= 1) return LEFT;
  return LEFT + index * WIDTH / (total - 1);
}

// Draws one trace in the band between `top` and `bottom` (pixels), scaled so
// `low` sits on the bottom edge and `high` on the top edge.
static void draw_trace(const float* values, int total, int top, int bottom, float low, float high,
                       vex::color trace_color) {
  if (high - low < 0.0001) high = low + 1;
  float scale = (bottom - top) / (high - low);

  // Zero line
  int zero_y = bottom - (int)((0 - low) * scale);
  Brain.Screen.setPenColor(vex::color(0x616161));
  Brain.Screen.drawLine(LEFT, zero_y, LEFT + WIDTH, zero_y);

  Brain.Screen.setPenColor(trace_color);
  for (int i = 1; i < total; i++) {
    int y1 = bottom - (int)((values[i - 1] - low) * scale);
    int y2 = bottom - (int)((values[i] - low) * scale);
    Brain.Screen.drawLine(column(i - 1, total), y1, column(i, total), y2);
  }
}

// Draws the whole graph: a title, the numbers, then the two traces.
static void draw() {
  int total = count;
  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(vex::color::black);
  Brain.Screen.setFont(vex::fontType::mono20);
  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.printAt(8, 18, true, "%s  %.2f s  %s", label, duration_ms() / 1000.0,
                       recording ? "" : "(tap for autons)");
  Brain.Screen.setPenColor(vex::color(0xBDBDBD));
  Brain.Screen.printAt(8, 40, true, "start %.1f  over %.1f  end %.1f %s", first_error, worst_overshoot,
                       last_error, units);
  if (total < 2) return;

  // Error, scaled to fit, always including zero.
  float low = 0, high = 0;
  for (int i = 0; i < total; i++) {
    if (errors[i] < low) low = errors[i];
    if (errors[i] > high) high = errors[i];
  }
  float margin = (high - low) * 0.08;
  draw_trace(errors, total, 50, 160, low - margin, high + margin, vex::color(0x66BB6A));

  // Voltage, always -12 to 12.
  draw_trace(outputs, total, 172, 232, -12.5, 12.5, vex::color(0xFFA726));

  Brain.Screen.setPenColor(vex::color(0x66BB6A));
  Brain.Screen.printAt(400, 64, false, "error");
  Brain.Screen.setPenColor(vex::color(0xFFA726));
  Brain.Screen.printAt(400, 186, false, "volts");
  Brain.Screen.setPenColor(vex::color::white);
}

// The drawing task. It redraws at most 5 times a second while a motion
// runs, once more when it ends, and gives the screen back when tapped.
static int graph_task() {
  bool was_touching = false;
  uint32_t last_draw = 0;
  while (true) {
    if (!enabled || !showing) {
      was_touching = false;
      vex::task::sleep(50);
      continue;
    }
    uint32_t now = now_ms();

    // A motion is over once it has stopped sending points.
    if (recording && now - last_record_ms > 80) {
      recording = false;
      dirty = true;
    }
    if (dirty && (!recording || now - last_draw >= 200)) {
      dirty = false;
      last_draw = now;
      draw();
    }

    // Letting go of the screen hands it back to the auton selector.
    bool touching = Brain.Screen.pressing();
    if (was_touching && !touching && !recording) showing = false;
    was_touching = touching;

    vex::task::sleep(20);
  }
  return 0;
}

void start() { static vex::task task(graph_task); }

}  // namespace graph
}  // namespace hh
