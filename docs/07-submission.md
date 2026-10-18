# 7. Submission

**Deadline: 18 October 2026, 23:59 (SGT).**

---

## 7.1 What to submit

A git repository containing this repository plus your work, with:

1. Your driver in `race_ws/src/team_driver/`, runnable as
   `ros2 run team_driver driver`.
2. Any extra packages declared in
   `race_ws/src/team_driver/requirements.txt` (pip) and
   `race_ws/src/team_driver/apt-packages.txt` (apt). Declared packages get
   installed into the image; undeclared ones will not exist during judging.
3. Anything you built offline — an occupancy grid, a racing line, trained
   weights — committed alongside, plus the code that produced it (rule 36).
4. A `SUBMISSION.md` at the repository root (template in §7.3).
5. Nothing else changed in the judging environment or the AutoDRIVE Devkit —
   verify with `./scripts/verify_judging_env.sh`.
6. **The results of your own judged runs**, in `results/submitted/` — the
   result files `./scripts/evaluate.sh` wrote on your machine, unedited, with
   the machine described in `SUBMISSION.md`. See below.

Do **not** commit `simulator/`, `race_ws/build/`, `race_ws/install/`,
`race_ws/log/`, or anything in `results/` other than `results/submitted/`. The
`.gitignore` already handles this. The simulator in particular is 140 MB of
binary that everyone fetches with `./scripts/fetch_simulator.sh`; committing it
will make your repository unusable rather than helpful.

### Your own judged runs

We race every entry ourselves on the judging machine, and that run is the one
that scores. Your own result files are the control: they record what the same
code did on your hardware, which is what lets us tell a real difference from a
broken one.

```sh
mkdir -p results/submitted
./scripts/evaluate.sh --team <your_team> --runs 3
cp results/<your_team>__*.json results/submitted/
git add results/submitted
```

What is required:

- **At least one run** with `"status": "COMPLETE"` and `"scored": true`. Commit
  as many as you like — the more we have, the better we can tell ordinary
  variation from something wrong.
- **The files exactly as the referee wrote them.** Do not edit them, do not
  assemble one by hand, do not rename a field. They are read against the format
  in [§5.5](05-evaluation.md#55-reading-a-result-file) and against our own run,
  and rule 50 applies to a result file as much as to a run.
- **The machine that produced them**, described in `SUBMISSION.md`: CPU, GPU,
  RAM, OS, and the `environment.real_time_factor` you usually saw.

If nothing you have run reaches `COMPLETE`, commit the best attempt you have
and say so in `SUBMISSION.md`. A `DNF` you are honest about costs you nothing
here; a missing file leaves us guessing.

**If our result and yours differ significantly, we will come to you.** A large
gap is usually environmental — a dependency that resolved to a different
version, a timing assumption that only holds on your hardware, a real-time
factor a long way from 1.0 — and we would rather debug it with you than record
a number neither of us believes. That is what this deliverable is for. It is
not a second leaderboard: nothing you commit here is scored.

The **Judge** workflow result is not a substitute. It runs on GitHub's
hardware, which is neither your machine nor ours.

### Model weights

If your entry is a learned policy, the weights have to be in the repository —
there is no network access during a judged run. Keep them to something
reasonable; if they are large enough that Git LFS is the only sane answer, say
so in `SUBMISSION.md` and check the judges can clone it.

---

## 7.2 How to submit

1. Fork this repository, or push a branch to the repository the organisers gave
   your team.
2. Push everything before the deadline.
3. Send the organisers your repository URL and the branch name.

### Test from a clean clone first

This is the single most common way to lose points. Your machine has state your
submission does not.

```sh
cd /tmp
git clone <your repo url> submission_test
cd submission_test
./scripts/fetch_simulator.sh
./install/linux/setup.sh --no-cache   # picks up your declared packages
./install/linux/run.sh
# inside the container
cd /hackathon/race_ws && colcon build --symlink-install && source install/local_setup.bash
exit
./scripts/evaluate.sh --team clean_clone_test --runs 1
```

`--no-cache` matters if you declared any packages: it proves they install from
scratch and that they did not break the devkit's websocket stack. The build
fails loudly if they did — and read rule 37 before you pin anything named
`socketio`, `engineio` or `gevent`.

If that produces a `COMPLETE` result, your entry will run on judging day.

Pushing runs the same thing on GitHub Actions and writes the result to the
workflow summary — see
[§5.6](05-evaluation.md#56-automated-judging-on-every-push). Check it is green
before you tell us you are done — green means your entry runs, not that it is
fast, and none of the times it prints are scored.

---

## 7.3 `SUBMISSION.md` template

```markdown
# Team JfK

## Members
| Name | Matric number | Email |
| Kow Zi Heng Nicholas | U2620646J | zkow002@e.ntu.edu.sg |
| Benjamin Lim | U2620145L | blim092@e.ntu.edu.sg |
| Chok Jun Xian | U2620924L | jchok002@e.ntu.edu.sg |
| Liew Cheng Hou | U26 | liew0266@e.ntu.edu.sg |
| Kam Wei Hong | U2623105F | wkam002@e.ntu.edu.sg |

## Approach
Two or three paragraphs: what your driver does, why you chose it, what you
tried that did not work.

## How it uses the inputs
- LiDAR: ...
- Pose: ... (say whether you used the simulator's ground truth or your own
  localisation)
- Camera / IMU / encoders: ... (or "not used")

## Speed control
How you turn a desired speed into a throttle command, and how well it holds.
"We used the shipped SpeedController unchanged" is a perfectly good answer —
just say so.

## Results on our own machine
The runs committed in `results/submitted/`, summarised.

| Best lap | 10-lap total | Collisions | Runs attempted |
| --- | --- | --- | --- |
| ... | ... | ... | ... |

### The machine that produced them
| CPU | GPU | RAM | OS | Typical real-time factor |
| --- | --- | --- | --- | --- |
| ... | ... | ... | ... | ... |

Anything else that might explain a gap between your numbers and ours: a run
that only works on the second attempt, a warning the referee logged, a result
you cannot reproduce.

## Anything precomputed
A map, a racing line, trained weights. What is computed offline, which script
produces it, and how to regenerate it from scratch. For trained weights, say
how they were trained as well — environment or dataset, reward or loss, roughly
how long, and how you decided the result was good. That description is what we
discuss at the interview, not the weights themselves.

## Dependencies we added
Everything in `requirements.txt` / `apt-packages.txt`, with one line each on
what it is for. Write "none" if you added nothing.

| Package | pip or apt | What we use it for |
| --- | --- | --- |
| ... | ... | ... |

## Third-party code and references
Papers, repositories, blog posts you took ideas or code from.

## AI assistance
Which tools you used and for what. This is permitted — we ask so the interview
can focus on the parts you wrote yourself.

## Known issues
Where it breaks, and what you would do next with more time.
```

---

## 7.4 The interview

Every team is interviewed after the race. Expect roughly 30 minutes.

**What we ask about:**

- Walk us through your driver, in your own words.
- Why that approach and not the others?
- What does this parameter do, and what happens if we double it?
- Show us a change you made that did not work, and how you knew.
- If any of it is learned: how did you train it, why that way, and how do you
  know it works?
- How does your throttle become a speed? What happens when it is wrong?
- How would you handle a track twice as fast? An obstacle mid-corner?
- You have never seen this circuit. What in your design made that survivable?

That last one is the question this track exists to ask. The compete circuit was
unseen by everyone, so an entry that generalises is worth more than one tuned
to death against something else.

**Any member may be asked about any part of the code.** Splitting the work is
fine; not knowing what your teammates built is not.

**Bonus marks** are available for replacing the algorithm outright (RL, MPC,
imitation learning), mapping the circuit yourself, your own localisation,
runtime racing-line generation, a real speed controller, and clear
engineering — see [rule 45](06-rules.md#69-interview-and-bonus-marks). These
affect the recruitment decision, not the leaderboard.

---

## 7.5 Submission checklist

- [ ] 3 to 5 members, all named in `SUBMISSION.md`
- [ ] `ros2 run team_driver driver` works from a clean clone
- [ ] Any extra packages declared in `requirements.txt` / `apt-packages.txt`
      **and** listed in `SUBMISSION.md`
- [ ] Nothing pinned that clashes with the devkit's websocket stack (rule 37)
- [ ] `./install/<os>/setup.sh --no-cache` succeeds from clean if you declared any
- [ ] `./scripts/verify_judging_env.sh` reports the environment intact
- [ ] `./scripts/evaluate.sh --team <team> --runs 1` gives `COMPLETE`
- [ ] Your own judged runs committed in `results/submitted/`, unedited, with
      the machine described in `SUBMISSION.md`
- [ ] The **Judge** workflow is green on your submitted branch (a check that
      your entry runs, not a score — the organisers' run decides that)
- [ ] No build artefacts or simulator committed, and no result files outside
      `results/submitted/`
- [ ] Anything precomputed is committed, with the code that generated it
- [ ] `SUBMISSION.md` filled in, including AI usage and attributions
- [ ] Pushed, and the URL sent to the organisers, before 18 Oct 2026 23:59 SGT

---

Back to the **[README](../README.md)**.
