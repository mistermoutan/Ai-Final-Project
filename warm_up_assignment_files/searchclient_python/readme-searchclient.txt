/*******************************************************\
|                AI and MAS: Searchclient               |
|                        README                         |
\*******************************************************/

This readme describes how to use the included Python searchclient with the server that is contained in server.jar. 

The Python searchclient has been tested with Python 3.7, but may work with some previous versions.
The searchclient requires the 'psutil' package to monitor its memory usage; the package can be installed with pip:
    $ pip install psutil

All the following commands assume the working directory is the one this readme is located in.

You can read about the server options using the -? argument:
    $ java -jar ../server.jar -?

Starting the server using the searchclient:
    $ java -jar ../server.jar -l ../levels/SAD1.lvl -c "py searchclient/searchclient.py" -g 150 -t 300

The searchclient uses the BFS search strategy by default. Use arguments -dfs, -astar, -wastar, or -greedy to set
alternative search strategies (after you implement them). For instance, to use DFS search on the same level as above:
    $ java -jar ../server.jar -l ../levels/SAD1.lvl -c "py searchclient/searchclient.py -dfs" -g 150 -t 300

Memory settings:
    * Unless your hardware is unable to support this, you should let the searchclient allocate at least 2GB of memory *
    The searchclient monitors its own process' memory usage and terminates the search if it exceeds a given number of MiB.
    To set the max memory usage to 2GB (which is also the default):
        $ java -jar ../server.jar -l ../levels/SAD1.lvl -c "py searchclient/searchclient.py --max-memory 2048" -g 150 -t 300
    Avoid setting max memory usage too high, since it will lead to your OS doing memory swapping which is terribly slow.

Rendering on Unix systems:
    We experienced poor performance when rendering on some Unix systems, because hardware rendering is not turned on by default.
    To enable OpenGL hardware acceleration you should use the following JVM option: -Dsun.java2d.opengl=true
        $ java -Dsun.java2d.opengl=true -jar ../server.jar -l ../levels/SAD1.lvl -c "py searchclient/searchclient.py" -g 150 -t 300
    See http://docs.oracle.com/javase/8/docs/technotes/guides/2d/flags.html for more information.
