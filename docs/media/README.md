# docs/media/

Assets embedded in the top-level [README](../../README.md).

## `barn_worlds_strip.png` — worlds-difficulty strip

Place the side-by-side strip of BARN worlds (e.g. worlds 5 / 57 / 109 / 193 /
244) here with exactly this filename; the README already references it.

- If you captured the frames yourself: screenshot Gazebo (`gui:=true`) per
  world, then stitch, e.g.
  `ffmpeg -i w5.png -i w57.png ... -filter_complex hstack=inputs=5 barn_worlds_strip.png`
  (or ImageMagick: `convert +append w*.png barn_worlds_strip.png`).
- If you use the figure from the official BARN challenge materials, credit it
  in the README caption (Xiao et al., BARN dataset).

Keep it under ~2 MB (`convert barn_worlds_strip.png -resize 2000x png:- | ...`
or export as JPEG) so the README loads fast.

## Trial videos (simple + complex world)

**GitHub does not render committed `.mp4` files inside a README** — a relative
link just shows as a download link. To get an inline player:

1. Record the RViz2 window during a trial, e.g.

   ```bash
   # inside the distrobox, window capture via ffmpeg (adjust geometry/display):
   ffmpeg -f x11grab -framerate 30 -video_size 1280x720 -i :0.0+100,100 \
     -c:v libx264 -crf 28 -pix_fmt yuv420p trial_world0.mp4
   ```

   (or use OBS / GNOME screen recorder, then compress with
   `ffmpeg -i in.mp4 -c:v libx264 -crf 28 -pix_fmt yuv420p out.mp4`)

2. Open the README in GitHub's **web editor** and drag-and-drop the `.mp4`
   into the video placeholder cell. GitHub uploads it and inserts a
   `https://github.com/user-attachments/assets/...` URL, which renders as an
   inline player. Replace the *(video placeholder)* text with that URL on its
   own line.

3. Alternative that works with committed files only: convert short clips to
   GIF/webp (`ffmpeg -i trial.mp4 -vf "fps=12,scale=640:-1" trial.gif`) and
   embed like an image — but expect large files; prefer the upload flow.

Suggested runs:

```bash
# simple world
ros2 launch jackal_helper BARN_runner.launch.py \
  algo_type:=classical_mpc world_idx:=5 gui:=true planner_rviz:=true
# complex world
ros2 launch jackal_helper BARN_runner.launch.py \
  algo_type:=classical_mpc world_idx:=244 gui:=true planner_rviz:=true
```
