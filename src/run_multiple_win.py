import os
import subprocess
main_folder = "comp19"
run_cmd = 'java -jar server.jar -c "python searchclient_mas/searchclient.py -par" -l "{}" -g'


# Comp17 issues
# MA
# TODO: MAGroupOne: hard but maybe possible by doing random stuff and hoping it becomes solved
# TODO: MAHALnineK: solved, but slow realizer

# SA
# TODO:                SA dont waste time putting agent in storage
# TODO: SAEvilCorp     Failure unclear
# TODO: SAgroupname:   realizer and planner very slow
# TODO: SAFooBar:      unsolved, sometimes solved depending on goal order strategy
# TODO: SAGroupOne:    unsolved, similar to FooBar
# TODO: SAHALnineK:    wont be solved by our solution, need new strategy
# TODO: SALiquorice:   very slow realizer, weird output plan
# TODO: SAMasters:     stupid failure, rotation point blocked off (similar to SAFooBar)
# TODO: SAOmnics:     very slow realizer

# Comp18 issues
# MA
# TODO: MAAiMasTers: same as SAHALnineK
# TODO: MAKarlMarx: same as SAFooBar

# SA
# no issues except slow realizer and sometimes subpar plans

# Comp19
# General: agents seem to trap themselves sometimes when they dont have to
# MAbob: unsolved: Agent goal adjacent to final goal
# MAForThePie: does some weird jiggles that waste 1-2 moves occasionally probably due to heuristic in SA solution
# MAMASAI: might be solvable by doing random moves
# MANOAsArk: fails for unknown reason: maybe goal pullable?
# MASoulman: extremely slow realizer, seems unsolvable (at least consistently)
# SAAvicii: unknown failure: should be easy to solve
# SAdeepurple: bad plan
# SAgTHIRTEEN: bad storage values
# SANulPoint: bad plan: when clearing try to clear stuff close to agents?
# SARegExAZ: stupid ending, solves goal but keeps moving
# SAWallZ: known failure: may be fixable with random moves

# after improvement:
# MAbob: unsolved: Agent goal adjacent to final goal (fixable?)
# MAgTHIRTEEN: sliding puzzle
# comp19\MASoulman.lvl: unlucky goal picked (fixable?)
# MAVisualKei
# SAgTHIRTEEN: bad storage (fixable?)


# SAMASAI: weird behaviour due to search (still solved)
# SANulPoint: bad plan: when clearing try to clear stuff close to agents?

# MAGruppeTo: left agents in goals
# MARegExAZ: left agents in goals

if __name__ == '__main__':
    for sub in os.listdir(main_folder):
        if sub[-4:] == ".lvl":
            file = os.path.join(main_folder, sub)
            os.system(run_cmd.format(file))