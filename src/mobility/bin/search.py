import sys
from mobility.srv import Controller 

'''Searcher node.''' 

def run(state):
    pass 

def main():
    if len(sys.argv) < 2 :
        print 'usage:', sys.argv[0], '<rovername>'
        exit (-1)
    
    rover = sys.argv[1]
    rospy.init_node(rover + '_searcher')
    s = rospy.Service('searcher', Controller, run) 

if __name__ == '__main__' : 
    main()

