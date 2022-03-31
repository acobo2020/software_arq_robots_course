#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <softarq_msgs/SequenceAction.h>

typedef actionlib::SimpleActionClient<softarq_msgs::SequenceAction> Client;

class MyNode
{
public:
	MyNode()
	: ac("sequence", true)//el true levanta un thread que va a estar haciendo spin para ver los mensajes. importante los spins para las comunicacione
	{																															//sea cual sea en ros
		//ROS_INFO("Waiting for action server to start.");
		ac.waitForServer();
		//ROS_INFO("Action server started, sending goal.");
	}

	void doWork(long int until)
	{
		softarq_msgs::SequenceGoal goal;
		goal.end_limit = until;

		ac.sendGoal(goal,
				boost::bind(&MyNode::doneCb, this, _1, _2),
				Client::SimpleActiveCallback(),
				boost::bind(&MyNode::feedbackCb, this, _1));

	}

	void feedbackCb(const softarq_msgs::SequenceFeedbackConstPtr& feedback)//feedback que va a ir dando al cliente
	{
		ROS_INFO("Current count %ld", feedback->current);
	}

	//callback al que va a llamar cuando acabe la accion
	void doneCb(const actionlib::SimpleClientGoalState& state,
			const softarq_msgs::SequenceResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Answer: %ld", result->last);
		ros::shutdown();//salimos del spin cuando ya hemos acabado
	}

private:
	Client ac;
};

int main (int argc, char **argv)
{
	ros::init(argc, argv, "sequence_client");
	ros::NodeHandle n;

	MyNode my_node;
	my_node.doWork(200);

	ros::spin();

	return 0;
}
