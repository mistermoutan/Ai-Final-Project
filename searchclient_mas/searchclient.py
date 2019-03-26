#import argparse
#import re
import sys
import traceback



class SearchClient:
    def __init__(self, server_messages):
        #Adapt to data structure and split msg in parts.
        self.levelname = None
        self.colors = None
        self.init = None
        self.goal = None
        try:
            line = server_messages.readline().rstrip()

            while line:
                #store msg in datastructure
                print(line, file=sys.stderr, flush=True)
                if line.rstrip()=="#end":
                    break
                line = server_messages.readline().rstrip()

        except Exception as ex:
            print('Error parsing level: {}.'.format(repr(ex)), file=sys.stderr, flush=True)
            print(traceback.format_exc(), file=sys.stderr, flush=True)
            sys.exit(1)
    
        self.sendComment("Initialized SearchClient")
    def search(self, strategy: 'Strategy'):
        pass
    def sendAction(self,action):
        sys.stdout.write(str(action)+"\n")
        sys.stdout.flush()
    def sendComment(self,comment):
        sys.stdout.write("#"+str(comment)+"\n")
        sys.stdout.flush()
    
def main():
    #implement parse aguments if different planing algorithms are planned
    sys.stdout.write("ExampleClient\n")
    sys.stdout.flush()
    server_messages = sys.stdin
    client = SearchClient(server_messages)
if __name__ == '__main__':
    # Run client.
    main()

