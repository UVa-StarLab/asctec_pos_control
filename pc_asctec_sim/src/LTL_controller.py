#!/usr/bin/env python
import logging, rospy, time

from pc_asctec_sim.msg import pc_traj_cmd
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from tulip import spec
from tulip import synth
from tulip.transys import machines

global strategy, step, park_sig, state, start, space
global viz_pub, traj_pub

# Discretized space and room dimensions
x = 1.0
y = 2.0
t = 2

rows = 3
cols = 5
n = rows*cols #total cells

"""
    +-------+-------+-------+-------+
    |X(r-1)c|  ...  |  ...  | Xrc-1 |
    +-------+-------+-------+-------+
    |  ...  |       |       |  ...  |
    +-------+-------+-------+-------+
    |  Xc   | Xc+1  |  ...  | X2c-1 |
    +-------+-------+-------+-------+
    |  X0   |  X1   |  ...  |  Xc-1 |
    +-------+-------+-------+-------+

    
    < ---------------   y   ---------------->
    +-------+-------+-------+-------+-------+ <
    |	    |	    |	    |       |       | |
 -0.5  X10  |  X11  |  X12  |  X13  |  X14  | |
    |	    |	    |	    |       |       | |
    +-------+-------+-------+-------+-------+ |
    |	    |	    |	    |       |       | 
  0.0  X5   |  X6   |  X7   |  X8   |  X9   | x
    |	    |	    |	    |       |       | 
    +-------+-------+-------+-------+-------+ |
    |	    |	    |	    |       |       | |
  0.5  X0   |  X1   |  X2   |  X3   |  X4   | |
    |	    |	    |	    |       |       | |
    +-------+-------+-------+-------+-------+ <
      -1.0    -0.5     0.0     0.5     1.0

The robot is allowed to transition between any two adjacent cells;
diagonal motions are not allowed.
"""
def parkCallback(data):
	global park_sig
	park_sig = data.data

def joyCallback(data):
	global start, park_sig
	if data.buttons[0]:
		start = not start

	if data.buttons[1]:
		park_sig = not park_sig

def trajCallback(data):
	global start
	if start and data.data:
		global step, strategy, state, park_sig
		cmd = pc_traj_cmd()

		
		result = machines.guided_run(strategy,state,{'park': [park_sig]})
		state = result[0][0]
		step+=1
		print step, result[0][0],
		for key in result[1]:
			if key != 'X0reach' and key != 'X7reach':		
				if result[1][key].pop():
					print key,
					now = int(key[1:])

		print 'X7reach:', result[1]['X7reach'].pop(),
		print 'Park:', park_sig

		cmd.x[0] = (now // cols)*(x/(rows-1)) - x/2
		cmd.y[0] =  (now % cols)*(y/(cols-1)) - y/2
		cmd.z[0] = 1
		cmd.wait_time[0] = 0.25
		cmd.points = 1
		cmd.duration[0] = t
		traj_pub.publish(cmd)

def buildBorder():
	space = Marker()
	corner = Point()
	
	space.header.frame_id = '/odom'
	space.header.stamp = rospy.get_rostime()
	space.id = 2
	space.action = visualization_msgs.Marker.ADD
	space.type = visualization_msgs.Marker.LINE_LIST
	space.color.a = 1.0
	space.color.g = 1.0

	space.scale.x = 0.05
	space.scale.y = 1.0
	space.scale.z = 0.5

#	for i in xrange(n):
#		if i % cols != 0:
#
#		if i // cols != 0:

	viz_pub.publish(space)
	

if __name__ == '__main__':

	global start
	start = True

	# Environment assumptions
	env_vars = {'park'}
	env_init = set()                # empty set
	env_safe = set()                # empty set
	env_prog = '!park'            # []<>(!park)

	sys_vars = set(['X'+str(i) for i in xrange(n) ])
	sys_init = {'X0'}

	sys_safe = set()
	for i in xrange(n):
		first = True
		temp = 'X'+str(i)+' -> X (X'
		if i % cols != 0:
		#not leftmost column
			if first:
				first = False
				temp += str(i-1)
			else:
				temp += ' || X' + str(i-1)

		if (i+1) % cols != 0:
		#not rightmost column
			if first:
				first = False
				temp += str(i+1)
			else:
				temp += ' || X' + str(i+1)

		if i >= cols:
		#not bottom row
			if first:
				first = False
				temp +=	str(i-cols)
			else:
				temp += ' || X' + str(i-cols)

		if i < (n-cols):
		#not top row
			if first:
				first = False
				temp +=	str(i+cols)
			else:
				temp += ' || X' + str(i+cols)

		temp += ')'
		sys_safe.add(temp)	
	sys_safe.add(synth.exactly_one(sys_vars)[0])

	sys_prog = set()                # empty set

	sys_vars |= {'X7reach'}
	sys_init |= set()
	sys_safe |= {'(park -> X7reach)'}
	sys_prog |= {'X0', 'X14'}

	# Create a GR(1) specification
	specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
		            env_safe, sys_safe, env_prog, sys_prog)

	specs.moore = True
	specs.qinit = '\E \A'  # i.e., "there exist sys_vars: forall sys_vars"

	now = time.clock()
	strategy = synth.synthesize('omega', specs)
	assert strategy is not None, 'unrealizable'
	print 'Strategy took', time.clock() - now, 'seconds'

#	if not strategy.save('machine.png'):
#	    print(strategy)

	park_sig = False
	step = 0
	result = machines.guided_run(strategy,'Sinit',{'park': [park_sig]})
	state = result[0][0]
	print step, result[0][0],
	for key in result[1]:
		if key != 'X0reach' and key != 'X7reach':		
			if result[1][key].pop():
				print key,
	print 'X7reach:', result[1]['X7reach'].pop(),
	print 'Park:', park_sig

	rospy.init_node('LTL_Controller', anonymous=True)
	traj_pub = rospy.Publisher('/hummingbird_2/traj_points', pc_traj_cmd, queue_size=10)
	start_pub = rospy.Publisher("hummingbird_2/start",Bool, queue_size=10)
	viz_pub = rospy.Publisher("hummingbird_2/space",Marker, queue_size=10)

	rospy.Subscriber('/hummingbird_2/traj_end', Bool, trajCallback)
	rospy.Subscriber('/hummingbird_2/park_sig', Bool, parkCallback)
	rospy.Subscriber('/joy', Joy, joyCallback)

	while not rospy.is_shutdown():
		if start:
			start_pub.publish(True)
			cmd = pc_traj_cmd()	
			cmd.z[0] = cmd.z[1] = 1
			cmd.x[1] = -0.5
			cmd.y[1] = -1.0
			cmd.wait_time[0] = cmd.wait_time[1] = 1

			cmd.duration[0] = cmd.duration[1] = 4
			cmd.points = 2
			traj_pub.publish(cmd)
			rospy.spin()

