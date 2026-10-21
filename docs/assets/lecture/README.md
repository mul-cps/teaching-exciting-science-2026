# Lecture visual sources

The deck intentionally separates an illustrated concept, actual equipment, actual software views, and native editable teaching diagrams. It uses no archive concept JPG as a hardware photograph. All build assets are committed here; the builder does not read private evidence paths.

## Concept illustration

`factory-concept-hero.png` is the single AI-created visual in the new deck. It was created with the built-in image-generation tool on 21 October 2026. Slide 1 visibly labels it **Concept illustration**. It describes the sorting idea, not the actual hardware. The exact prompt and mode are retained in `factory-concept-hero.prompt.md`.

## Actual station photograph

`station-current.png` is the unmodified supplied station camera capture from 21 October 2026. Original source: `pick-place-resume/restored-station/fresh-home-capture/color.png` in the station evidence. Original SHA256: `b9c125b0eef72e432796aa5efa7c9024461873d0478d3acfe88f058e27c9bb1d`.

The following are crops of that photograph, with coordinates in the original 1920 × 1080 image:

| Asset | Crop size | Top-left |
|---|---|---|
| `arm-current.png` | 610 × 835 | 1270, 0 |
| `brick-and-gripper.png` | 850 × 650 | 900, 355 |
| `brick-red.png` | 68 × 68 | 1153, 593 |
| `brick-green.png` | 100 × 100 | 1290, 635 |
| `brick-red-shadow.png` | 75 × 75 | 1114, 644 |
| `colored-bricks-current.png` | 380 × 310 | 1060, 475 |
| `leader-current.png` | 600 × 700 | 20, 380 |

`brick-pixels.png` reduces `brick-red.png` to 10 × 10 and enlarges it to 420 × 420 with nearest-neighbor sampling. It demonstrates pixels and does not represent native camera resolution. Extra actual crops support the resource activities; not every crop appears in the final slides.

The camera is outside its own view. There are no sorting bins in this photograph. Native destination diagrams represent the simulation convention. Photographs do not establish autonomous physical sorting success.

## Actual station screen

`digital-twin-screen.png` and `digital-twin-view.png` preserve the user-supplied actual side-view screenshot originally provided as `codex-clipboard-5CgWs7.png`. It shows the whole arm, its gripper, the measured scene, and the screen model. Slide 6 preserves its aspect ratio and identifies it as an actual station screen. It is a saved screenshot, not proof of a current live connection. The earlier green-box-dominated clipboard view is not used.

## Actual Blockly screen

`blockly-screen.png` is the actual Chrome view captured while checking [Blockly Games Maze](https://blockly.games/maze?lang=en&level=1&skin=0) on 21 October 2026. It shows level 2 after the first level's block/run check. `blockly-view.png` is a 980 × 550 crop at 0, 0, used without distortion on the activity checkpoint. Students start at level 1.

Source project: [Blockly Games](https://github.com/blockly-games/blockly-games). Its [Apache 2.0 license](https://github.com/blockly-games/blockly-games/blob/master/LICENSE) is copied unchanged as `Blockly-Games-LICENSE.txt`. The screenshot attributes the actual app and is included to identify the teaching activity, not as a claim that the app is part of this repository.

## Prepared Teachable Machine photo samples

All nine images in `tm-samples/` are cropped from `station-current.png` and resized to 224 × 224. They are deliberately small, correlated classroom samples. A training/inference cycle with them has not been verified, and they do not establish general recognition quality.

| Class | Original crops (width × height at x,y) |
|---|---|
| red | 92 × 92 at 1141,587; 75 × 75 at 1114,644; 84 × 84 at 1147,594 |
| blue | 90 × 90 at 1120,510; 80 × 80 at 1225,470; 77 × 77 at 1133,523 |
| background | 110 × 110 at 970,760; 90 × 90 at 1300,765; 90 × 90 at 1350,875 |

No video is embedded or downloaded. The verified optional Google video is linked in the speaker notes and resource sheet. The separate `../../../stale/docs/assets/archive/scene-*.jpg` files are AI-generated concepts and are not used in this deck.
