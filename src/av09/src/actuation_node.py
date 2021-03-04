

#Actuation_node


Class Act:
	def __init__(self,goal=NONE,vr=NONE,action=NONE):
		self.goal=goal
		self.vr=vr
		self.action=action

Class PLAN:
	def __init__(self);
		self.OC=1
		self.RC=NONE
		self.ACT=ACT()
	def callback(self, msg):
		self.ACT=msg.plan.act
		self.RC=msg.plan.rc

Class StateMachine:
	def __init__(self,state=1,plan=NONE):
		self.state=state
		self.plan=PLAN()
	
	def Stateswitch(self):
		return getattr(self, 'state_' + str(self.state), lambda: default)
				
	

	def state_1(self):
		self.OC=1
		if self.plan.ACT.action=='reroute':
			goal.push(self.plan.ACT.goal)
		if self.plan.RC=='AUTO':
			self.state=3
		elif self.plan.RC=='CT':
			self.state=2

	def state_2(self):
		self.OC=3
		if self.plan.ACT.action== 'reroute and safe':
			goal.push(self.plan.ACT.goal)
			vr.push(self.plan.ACT.vr)
		elif self.plan.ACT.action=='EB':
			vr.publish(0)

		if self.plan.RC=='Released':
			self.state=1
		elif self.plan.RC=='CT':
			self.state=2
	def state_3(self):

		self.OC=2
		if self.plan.ACT.action== 'reroute and safe':
			goal.push(self.plan.ACT.goal)
			vr.push(self.plan.ACT.vr)
		elif self.plan.ACT.action=='EB':
			vr.publish(0)

		if self.plan.RC=='Released':
			self.state=1
		elif self.plan.RC=='CT':
			self.state=2

def main():
	rospy.init_node(Actuation_node)
	
	
	SM=StateMachine(1)
	rospy.subscriber('\plan', msgClass, SM.plan.callback )
	SM.stateswitch()


	rospy.spin()



if __name__ == '__main__':
	main()