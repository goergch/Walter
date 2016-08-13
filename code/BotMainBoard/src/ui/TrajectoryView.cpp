/*
 * TrajectoryView.cpp
 *
 *  Created on: 12.08.2016
 *      Author: SuperJochenAlt
 */

#include <ui/TrajectoryView.h>
#include "MainBotController.h"
#include "BotWindowCtrl.h"


// live variables of controls
char trajectoryItemNameLiveVar[128] = "";
int trajectoryItemDurationLiveVar;
int trajectorySmoothLiveVar;

// IDs of GLUI controls
const int SaveButtonID 		= 0;
const int DeleteButtonID 	= 1;
const int UpButtonID 		= 2;
const int DownButtonID 		= 3;

// controls
GLUI_List* trajectoryList = NULL;

TrajectoryView::TrajectoryView() {
}

void trajectoryListCallback(int controlNo) {
	// a new list item has been selected, use this node as pose
	int idx = trajectoryList->get_current_item();
	vector<TrajectoryNode>& trajectory = TrajectoryView::getInstance().trajectory;
	TrajectoryNode currentNode = trajectory[trajectory.size()-idx-1];

	BotWindowCtrl::getInstance().setNewPose(currentNode.pose);
}

void trajectoryDurationCallback(int controlNo) {

}

void trajectoryNameCallback(int controlNo) {
}

void trajectorySmoothCallback(int controlNo) {

}

void TrajectoryView::fillTrajectoryListControl() {
	trajectoryList->delete_all();

	for (unsigned int i = 0;i<trajectory.size();i++) {
		int idx = trajectory.size()-i-1;
		TrajectoryNode node =  trajectory[idx];
		trajectoryList->add_item(trajectory.size()-i+1,node.getText().c_str());
	}
}


void trajectoryButtonCallback(int controlNo) {
	vector<TrajectoryNode>& trajectory = TrajectoryView::getInstance().trajectory;

	switch (controlNo) {
		case SaveButtonID: {
			// store current Pose
			TrajectoryNode node;
			node.pose = MainBotController::getInstance().getCurrentPose();
			node.name = trajectoryItemNameLiveVar;
			node.duration = trajectoryItemDurationLiveVar;
			node.smooth = (trajectorySmoothLiveVar != 1);
			int idx = trajectoryList->get_current_item();

			vector<TrajectoryNode>::iterator trajListIter = trajectory.begin();

			int insertAt = (trajectory.size()-idx);
			trajectory.insert(trajectory.begin() + insertAt, node);
			TrajectoryView::getInstance().fillTrajectoryListControl();
			break;
		}
		case DeleteButtonID: {
			if (trajectory.size() > 0) {
				int idx = trajectoryList->get_current_item();

				int deleteAt = (trajectory.size()-idx)-1;
				trajectory.erase(trajectory.begin() + deleteAt);
				TrajectoryView::getInstance().fillTrajectoryListControl();
				trajectoryList->set_current_item(idx);
			}
			break;
		}
		case UpButtonID:
			break;
		case DownButtonID:
			break;
	}
}



void TrajectoryView::create(GLUI *windowHandle, GLUI_Panel* interactivePanel) {
	GLUI_Panel* trajectoryPanel = new GLUI_Panel(interactivePanel,"trajectory panel", GLUI_PANEL_RAISED);
		new GLUI_StaticText(trajectoryPanel,"                          trajectory planning                           ");

		GLUI_Panel* trajectoryPlanningPanel = new GLUI_Panel(trajectoryPanel,"trajectory panel", GLUI_PANEL_NONE);

		trajectoryList = new GLUI_List(trajectoryPlanningPanel,"trajectory list", true, trajectoryListCallback);
		trajectoryList->set_h(90);
		trajectoryList->set_w(140);

		fillTrajectoryListControl();
		new GLUI_EditText( trajectoryPlanningPanel, "name", GLUI_EDITTEXT_TEXT, &trajectoryItemNameLiveVar, 0, trajectoryNameCallback );
		new GLUI_EditText( trajectoryPlanningPanel, "time[s]",GLUI_EDITTEXT_INT,  &trajectoryItemDurationLiveVar, 0, trajectoryDurationCallback);

		windowHandle->add_column_to_panel(trajectoryPlanningPanel, false);
		GLUI_Panel* trajectoryButtonPanel = new GLUI_Panel(trajectoryPlanningPanel,"trajectory  button panel", GLUI_PANEL_NONE);
		new GLUI_Button( trajectoryButtonPanel, "save", SaveButtonID,trajectoryButtonCallback );
		new GLUI_Button( trajectoryButtonPanel, "delete",DeleteButtonID,trajectoryButtonCallback  );
		new GLUI_Button( trajectoryButtonPanel, "up" ,UpButtonID,trajectoryButtonCallback );
		new GLUI_Button( trajectoryButtonPanel, "down",DownButtonID,trajectoryButtonCallback  );
		new GLUI_StaticText( trajectoryButtonPanel, "" );

		GLUI_Checkbox* smoothCheckBox = new GLUI_Checkbox( trajectoryButtonPanel, "smooth", &trajectorySmoothLiveVar, 0, trajectorySmoothCallback );
		smoothCheckBox->set_alignment(GLUI_ALIGN_CENTER);

		GLUI_Panel* trajectoryMovePanel = new GLUI_Panel(interactivePanel,"trajectory move panel", GLUI_PANEL_RAISED);
		GLUI_StaticText* headline=new GLUI_StaticText(trajectoryMovePanel,"                          trajectory execution                          ");
		headline->set_alignment(GLUI_ALIGN_CENTER);
		GLUI_Panel* trajectoryPlayPanel = new GLUI_Panel(trajectoryMovePanel,"trajectory move panel", GLUI_PANEL_NONE);
		GLUI_Panel* trajectoryRealPanel = new GLUI_Panel(trajectoryMovePanel,"trajectory real panel", GLUI_PANEL_NONE);
		new GLUI_Button( trajectoryPlayPanel, "forward" );
		new GLUI_Button( trajectoryPlayPanel, "play" );
		windowHandle->add_column_to_panel(trajectoryPlayPanel, false);
		new GLUI_Button( trajectoryPlayPanel, "back" );
		new GLUI_Button( trajectoryPlayPanel, "stop" );
		new GLUI_Checkbox(trajectoryRealPanel,"MOVE BOT");
}
