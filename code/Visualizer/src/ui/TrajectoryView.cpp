/*
 * TrajectoryView.cpp
 *
 *  Created on: 12.08.2016
 *      Author: SuperJochenAlt
 */

#include <ui/TrajectoryView.h>
#include "MainBotController.h"
#include "BotWindowCtrl.h"
#include "Trajectory.h"

using namespace std;

// live variables of controls
char trajectoryItemNameLiveVar[128] = "";
float trajectoryItemDurationLiveVar;
int interpolationTypeLiveVar;
int moveBotLiveVar;

vector<string> files;

// IDs of GLUI controls
const int InsertButtonID 		= 0;
const int OverwriteButtonID = 1;
const int DeleteButtonID 	= 2;
const int UpButtonID 		= 3;
const int DownButtonID 		= 4;
const int LoadButtonID 		= 5;
const int SaveButtonID 		= 6;
const int MergeButtonID 	= 7;

const int PlayButtonID 		= 11;
const int StopButtonID 		= 13;

// controls
GLUI_List* trajectoryList = NULL;
GLUI_List* fileSelectorList = NULL;
GLUI_EditText* nodeNameControl 		= NULL;
GLUI_Spinner*  nodeTimeControl 		= NULL;
GLUI_RadioGroup* interpolationTypeControl = NULL;
GLUI_StaticText* infoText 			= NULL;
GLUI_Panel* interactivePanel = NULL;
GLUI_FileBrowser* fileBrowser = NULL;
TrajectoryView::TrajectoryView() {
}

void TrajectoryView::display() {
	int idx = Trajectory::getInstance().selectedNode();
	if (idx >= 0)
		trajectoryList->set_current_item(Trajectory::getInstance().getTrajectory().size()-idx-1);
}


void fillfileSelectorList() {
	int idx = fileSelectorList->get_current_item();
	string selectedFile;
	if ((idx >= 0) && (idx < (int)files.size()))
		selectedFile = files[idx];

	files = readDirectory(".","trj");
	fileSelectorList->delete_all();
	for (unsigned i = 0;i<files.size();i++) {
		fileSelectorList->add_item(i, files[i].c_str());
		if (selectedFile == files[i])
			fileSelectorList->set_current_item(i);
	}
}

void fileSelectorListCallback(int controlNo) {
}

void TrajectoryView::idle() {

	// refresh file selector view every 5s
	static int timeExecution = -1;
	int time = (millis() / 1000) % 5;
	if ((time == 0) && (timeExecution != time)) {
		fillfileSelectorList();
		timeExecution = time;
	}
	if (timeExecution != time)
		timeExecution = -1;
}

void trajectoryListCallback(int controlNo) {
	// a new list item has been selected, use this node as pose
	int idx = trajectoryList->get_current_item();
	vector<TrajectoryNode>& trajectory = Trajectory::getInstance().getTrajectory();
	TrajectoryNode currentNode = Trajectory::getInstance().selectNode(trajectory.size()-idx-1);

	nodeTimeControl->set_float_val(((float)currentNode.duration_ms)/1000.0);
	interpolationTypeControl->set_int_val(currentNode.interpolationType);

	nodeNameControl->set_text(currentNode.name.c_str());

	// set pose of bot to current node
	MainBotController::getInstance().setAnglesImpl(currentNode.angles);
	MainBotController::getInstance().setPose(currentNode.pose);
}

void trajectoryDurationCallback(int controlNo) {
	static float lastValue;
	float value = trajectoryItemDurationLiveVar;
	float roundedValue = roundValue(value);
	if (roundedValue == lastValue) {
		roundedValue += sgn(value-lastValue)*0.1;
	}
	nodeTimeControl->set_float_val(roundedValue);
	lastValue = roundedValue;
}

void trajectoryNameCallback(int controlNo) {
}

void interpolationTypeCallback(int controlNo) {

}

void TrajectoryView::fillTrajectoryListControl() {
	trajectoryList->delete_all();

	vector<TrajectoryNode>& trajectory = Trajectory::getInstance().getTrajectory();

	for (unsigned int i = 0;i<trajectory.size();i++) {
		int idx = trajectory.size()-i-1;
		TrajectoryNode node =  trajectory[idx];
		trajectoryList->add_item(trajectory.size()-i+1,node.getText().c_str());
	}
}

void trajectoryButtonCallback(int controlNo) {
	vector<TrajectoryNode>& trajectory = Trajectory::getInstance().getTrajectory();

	switch (controlNo) {
		case InsertButtonID: {
			// store current Pose
			TrajectoryNode node;
			node.pose = MainBotController::getInstance().getCurrentPose();
			node.angles = MainBotController::getInstance().getCurrentAngles();

			node.name = trajectoryItemNameLiveVar;
			node.duration_ms = (int)(trajectoryItemDurationLiveVar*1000.0);
			node.interpolationType = (InterpolationType)interpolationTypeLiveVar;
			int idx = trajectoryList->get_current_item();

			vector<TrajectoryNode>::iterator trajListIter = trajectory.begin();

			int insertAt = (trajectory.size()-idx);
			trajectory.insert(trajectory.begin() + insertAt, node);
			TrajectoryView::getInstance().fillTrajectoryListControl();
			trajectoryList->set_current_item(idx);

			break;
		}
		case OverwriteButtonID: {
			if (trajectory.size() == 0)
				trajectoryButtonCallback(InsertButtonID);
			else {
				// store current Pose
				TrajectoryNode node;
				node.pose = MainBotController::getInstance().getCurrentPose();
				node.angles = MainBotController::getInstance().getCurrentAngles();
				node.name = trajectoryItemNameLiveVar;
				node.duration_ms = (int)(trajectoryItemDurationLiveVar*1000.0);
				node.interpolationType = InterpolationType(interpolationTypeLiveVar);

				int idx = trajectoryList->get_current_item();
				int overwriteAt = (trajectory.size()-idx-1);
				trajectory[overwriteAt] = node;
				TrajectoryView::getInstance().fillTrajectoryListControl();
				trajectoryList->set_current_item(idx);
			}
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
		case UpButtonID: {
			if (trajectory.size() > 1) {
				int controlIdx= trajectoryList->get_current_item();
				int trajIdx = (trajectory.size()-controlIdx-1);
				if ((unsigned)(trajIdx+1) < trajectory.size()) {
					TrajectoryNode currNode = trajectory[trajIdx];
					TrajectoryNode upNode= trajectory[trajIdx+1];
					trajectory[trajIdx] = upNode;
					trajectory[trajIdx+1] = currNode;
					TrajectoryView::getInstance().fillTrajectoryListControl();
					trajectoryList->set_current_item(controlIdx-1);
					trajectoryListCallback(0);
				}
			}
			break;
		}
		case DownButtonID: {
			if (trajectory.size() > 1) {
				int controlIdx= trajectoryList->get_current_item();
				int trajIdx = (trajectory.size()-controlIdx-1);
				if (trajIdx > 0) {
					TrajectoryNode currNode = trajectory[trajIdx];
					TrajectoryNode dnNode= trajectory[trajIdx-1];
					trajectory[trajIdx] = dnNode;
					trajectory[trajIdx-1] = currNode;
					TrajectoryView::getInstance().fillTrajectoryListControl();
					trajectoryList->set_current_item(controlIdx+1);
					trajectoryListCallback(0);

				}
			}
			break;
		case SaveButtonID: {
			// find a free filename
			vector<TrajectoryNode>& trajectory = Trajectory::getInstance().getTrajectory();
			if (trajectory.size() >=2) {
				string name = trajectory[0].name;
				string prefix;
				if (name.empty() || name=="") {
					prefix= "t";
				} else {
					prefix= name;
				}
				// make filename unique
				int i = 0;
				string filename = prefix + "-" + int_to_string(i);
				while (fileExists(filename+".trj")) {
					filename = prefix + "-" + int_to_string(i);
					i++;
				}
				// remove invalid characters
				Trajectory::getInstance().save(filename + ".trj");
				fillfileSelectorList();
			}
			break;
		}
		case LoadButtonID: {
			string filename;
			int idx = fileSelectorList->get_current_item();
			if (idx >= 0)
				filename = files[idx];
			if (!filename.empty()) {
				Trajectory::getInstance().load(filename);
				TrajectoryView::getInstance().fillTrajectoryListControl();
			}
			break;
		}
		case MergeButtonID: {
			string filename;
			int idx = fileSelectorList->get_current_item();
			if (idx >= 0)
				filename = files[idx];
			if (!filename.empty()) {
				Trajectory::getInstance().merge(filename);
				TrajectoryView::getInstance().fillTrajectoryListControl();
			}
			break;
		}

		default:
			break;
		}
	} // switch

	// compile trajectory ( timing and interpolation )
	Trajectory::getInstance().compile();
}

void trajectoryPlayerCallback (int controlNo) {
	switch (controlNo) {
	case StopButtonID: {
		MainBotController::getInstance().stopTrajectory();
		break;
		}
	case PlayButtonID: {
		MainBotController::getInstance().playTrajectory();
		break;
		}
	default:
		break;
	}
}

void TrajectoryView::create(GLUI *windowHandle, GLUI_Panel* pInteractivePanel) {
	interactivePanel = pInteractivePanel;
	GLUI_Panel* trajectoryPanel = new GLUI_Panel(interactivePanel,"trajectory panel", GLUI_PANEL_RAISED);
		new GLUI_StaticText(trajectoryPanel,"                          trajectory planning                           ");

	GLUI_Panel* trajectoryPlanningPanel = new GLUI_Panel(trajectoryPanel,"trajectory panel", GLUI_PANEL_NONE);
	trajectoryList = new GLUI_List(trajectoryPlanningPanel,"trajectory list", true, trajectoryListCallback);
	trajectoryList->set_h(115);
	trajectoryList->set_w(130);

	fillTrajectoryListControl();
    nodeNameControl = new GLUI_EditText( trajectoryPlanningPanel, "name", GLUI_EDITTEXT_TEXT, &trajectoryItemNameLiveVar, 0, trajectoryNameCallback );
	nodeTimeControl = new GLUI_Spinner( trajectoryPlanningPanel, "time[s]",GLUI_SPINNER_FLOAT,  &trajectoryItemDurationLiveVar, 0, trajectoryDurationCallback);
	nodeTimeControl->set_float_limits(0.1,10.0);
	nodeTimeControl->set_float_val(1.0);

	windowHandle->add_column_to_panel(trajectoryPlanningPanel, false);
	GLUI_Panel* trajectoryButtonPanel = new GLUI_Panel(trajectoryPlanningPanel,"trajectory  button panel", GLUI_PANEL_NONE);

	GLUI_Button* button = new GLUI_Button( trajectoryButtonPanel, "insert", InsertButtonID,trajectoryButtonCallback );
	button->set_w(70);
	button = new GLUI_Button( trajectoryButtonPanel, "overwrite", OverwriteButtonID,trajectoryButtonCallback );
	button->set_w(70);
	button = new GLUI_Button( trajectoryButtonPanel, "delete",DeleteButtonID,trajectoryButtonCallback  );
	button->set_w(70);
	button = new GLUI_Button( trajectoryButtonPanel, "up" ,UpButtonID,trajectoryButtonCallback );
	button->set_w(70);
	button = new GLUI_Button( trajectoryButtonPanel, "down",DownButtonID,trajectoryButtonCallback  );
	button->set_w(70);
	new GLUI_StaticText( trajectoryButtonPanel, "" );
	interpolationTypeControl = new GLUI_RadioGroup( trajectoryButtonPanel,&interpolationTypeLiveVar,0, interpolationTypeCallback);
	interpolationTypeControl->set_alignment(GLUI_ALIGN_CENTER);
	new GLUI_RadioButton( interpolationTypeControl, "linear" );
	new GLUI_RadioButton( interpolationTypeControl, "bezier" );
	new GLUI_RadioButton( interpolationTypeControl, "smooth" );
	interpolationTypeControl->set_int_val(InterpolationType::CUBIC_BEZIER);


	GLUI_Panel* trajectoryMovePanel = new GLUI_Panel(interactivePanel,"trajectory move panel", GLUI_PANEL_RAISED);
	GLUI_StaticText* headline=new GLUI_StaticText(trajectoryMovePanel,"                          trajectory execution                          ");

	headline->set_alignment(GLUI_ALIGN_CENTER);
	GLUI_Panel* trajectoryPlayPanel = new GLUI_Panel(trajectoryMovePanel,"trajectory move panel", GLUI_PANEL_NONE);

	// GLUI_Panel* trajectoryRealPanel = new GLUI_Panel(trajectoryMovePanel,"trajectory real panel", GLUI_PANEL_NONE);
	GLUI_Panel* trajectoryPlayButtonPanel = new GLUI_Panel(trajectoryPlayPanel,"trajectory move panel", GLUI_PANEL_NONE);

	button = new GLUI_Button( trajectoryPlayButtonPanel, "play", PlayButtonID, trajectoryPlayerCallback);
	button->set_w(70);
	button = new GLUI_Button( trajectoryPlayButtonPanel, "stop" , StopButtonID, trajectoryPlayerCallback);
	button->set_w(70);
	GLUI_Checkbox* moveBotCheckBox = new GLUI_Checkbox(trajectoryPlayButtonPanel,"for real", &moveBotLiveVar);
	moveBotCheckBox->set_alignment(GLUI_ALIGN_CENTER);

	windowHandle->add_column_to_panel(trajectoryPlayButtonPanel, false);
	button = new GLUI_Button( trajectoryPlayButtonPanel, "save" ,SaveButtonID,trajectoryButtonCallback );
	button->set_w(70);

	button = new GLUI_Button( trajectoryPlayButtonPanel, "load",LoadButtonID,trajectoryButtonCallback  );
	button->set_w(70);

	button = new GLUI_Button( trajectoryPlayButtonPanel, "merge",MergeButtonID,trajectoryButtonCallback  );
	button->set_w(70);
	windowHandle->add_column_to_panel(trajectoryPlayPanel, false);
	fileSelectorList = new GLUI_List(trajectoryPlayPanel,"trajectory list", true, fileSelectorListCallback);
	fileSelectorList->set_h(60);
	fileSelectorList->set_w(90);
	fillfileSelectorList();

}
