# SCUT Student Daily Life

## Group Members

- 王樾 (social force model, GUI improvement)
- 罗胤仪 (schedule generation)
- 陈晓 (map constuction)
- 韩龙啸 (GUI application)

## How to Build and Run

(The following ways are tested in Arch Linux)

### Qt Creator (Recommended)

Use Qt Creator to open the project (open CMakeLists.txt), use the default settings.

Then simply build and run.

### Custom (Need to manually modify relative address)

Suppose the directory is "SCUTStudentLife/build"

Globally search all relative address starting from "../SCUTStudentLife/", and substitude to "../".

Then simply build and run.

If assertion errors still occur due to `assert(ifs.is_open())`, check all these relative addresses.

## How to Use

By default, we import 3600 students as agents in the virtual map.

Firstly, set up the number of week, the day of week, the current time, the play rate at RHS. This step is necessary, and at least you must choose a time to apply.

It is recommended to set the play rate from 1 to 5 in order to avoid unexpected behaviors of agents.

Secondly, choose the options at LHS, such as whether to block down the first or second flyover or not.

After that, click on the "play" button to play, and you can pause the system by pressing the "pause" button when running.
