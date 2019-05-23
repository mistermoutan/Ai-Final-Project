import subprocess
from os import listdir
from os.path import isfile, join
import os

my_path = "./comp18/"
levels = [my_path + f for f in listdir(my_path) if isfile(join(my_path, f)) and ".lvl" in f]
print(levels)
#levels = ["levels/chokepoint.lvl", "levels/three_rooms.lvl"]

#subprocess.run(["ls", "-l"])
for level in levels:
    #os.system("java -jar server.jar -c 'python3 searchclient_mas/searchclient.py -par'  -l levels/three_rooms.lvl -g")
    subprocess.run(["java",  "-jar", "server.jar", "-c", "python3 searchclient_mas/searchclient.py -par", "-l" , level, "-g", "-s", "10"])
    

#run parallel planner
#java -jar src/server.jar -c 'python3 src/searchclient_mas/searchclient.py -par'  -l src/levels/three_rooms.lvl -g
    

#Command that I used to run the new server:
#NOTE 1: NAGIVATE TO TOP LEVEL FOLDER (the one outside src)
#NOTE 2: YOU HAVE TO DELETE OR RENAME GroupName.zip BEFORE YOU RUN THE COMMAND
#java -jar src/new_server.jar -c 'python3 src/searchclient_mas/searchclient.py -par' -l "src/comp19" -t 180 -o "GroupName.zip"