import os
import subprocess
main_folder = "comp17"
run_cmd = 'java -jar server.jar -c "python searchclient_mas/searchclient.py -par" -l "{}" -g'

# TODO: MAGeneralAI
# TODO: MAGroupOne
# TODO: MAHALnineK: agent clear
# TODO: MAOmnics
# TODO: MATALK ?????, was unsolvable, lol

# TODO: SAEvilCorp     SA levels in general have poor quality solutions
# TODO:                SA dont waste tiem putting agent in storage
# TODO: SAgroupname:   realizer and planner very slow
# TODO: SAGroupOne
# TODO: SALiquorice:   very slow realizer, weird output plan
# TODO: SAMasters:     stupid failure
# TODO: SA Omnics:     very slow realizer
if __name__ == '__main__':
    for sub in os.listdir(main_folder):
        if sub[-4:] == ".lvl":
            file = os.path.join(main_folder, sub)
            os.system(run_cmd.format(file))