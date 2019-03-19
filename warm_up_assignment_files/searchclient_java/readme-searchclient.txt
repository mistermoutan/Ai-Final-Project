/*******************************************************\
|                AI and MAS: Searchclient               |
|                        README                         |
\*******************************************************/

This readme describes how to use the included Java searchclient with the server that is contained in server.jar.
The search client should at least work with Java 8 (both Oracle JDK and OpenJDK), but you are also free to use a newer release.

Note that if you have the CLASSPATH environment variable set, the following commands may/will fail.
You should not have the CLASSPATH environment variable set unless you know what you're doing.

All the following commands assume the working directory is the one this readme is located in.

You can read about the server options using the -? argument:
    $ java -jar ../server.jar -?

Compiling the searchclient:
    $ javac searchclient/SearchClient.java

Starting the server using the searchclient:
    $ java -jar ../server.jar -l ../levels/SAD1.lvl -c "java searchclient.SearchClient" -g 150 -t 300

The searchclient uses the BFS search strategy by default. Use arguments -dfs, -astar, -wastar, or -greedy to set
alternative search strategies (after you implement them). For instance, to use DFS search on the same level as above:
    $ java -jar ../server.jar -l ../levels/SAD1.lvl -c "java searchclient.SearchClient -dfs" -g 150 -t 300

Memory settings:
    * Unless your hardware is unable to support this, you should let the JVM allocate at least 2GB of memory for the searchclient *
    Your JVM determines how much memory a program is allowed to allocate. These settings can be manipulated by certain VM options.
    The -Xmx option sets the maximum size of the heap, i.e. how much memory your program can allocate.
    The -Xms option sets the initial size of the heap.
    To set the max heap size to 2GB:
        $ java -jar ../server.jar -l ../levels/SAD1.lvl -c "java -Xmx2048m searchclient.SearchClient" -g 150 -t 300
        $ java -jar ../server.jar -l ../levels/SAD1.lvl -c "java -Xmx2g searchclient.SearchClient" -g 150 -t 300
    Note that this option is set in the *client*.
    Avoid setting max heap size too high, since it will lead to your OS doing memory swapping which is terribly slow.

Rendering on Unix systems:
    We experienced poor performance when rendering on some Unix systems, because hardware rendering is not turned on by default.
    To enable OpenGL hardware acceleration you should use the following JVM option: -Dsun.java2d.opengl=true
    Note that this JVM option must be set in the Java command that invokes the *server*:
        $ java -Dsun.java2d.opengl=true -jar ../server.jar -l ../levels/SAD1.lvl -c "java searchclient.SearchClient" -g 150 -t 300
    See http://docs.oracle.com/javase/8/docs/technotes/guides/2d/flags.html for more information.
