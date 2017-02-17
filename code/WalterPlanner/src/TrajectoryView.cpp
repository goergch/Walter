/*
 * TrajectoryView.cpp
 *
 * Author: JochenAlt
 */

#include <stdio.h>
#include "logger.h"

#include <TrajectoryView.h>
#include "TrajectorySimulation.h"
#include "BotWindowCtrl.h"
#include "ExecutionInvoker.h"
#include "Hanoi.h"

using namespace std;

// live variables of controls
char trajectoryItemNameLiveVar[128] = "";
int trajectoryItemSpeedLiveVar = 200;
int trajectoryItemDurationLiveVar = 0;

int interpolationTypeLiveVar;
int powerOnOffLiveVar;
int connectionToRealBotLiveVar;
vector<string> trajectoryFiles;

// all possible values for bot connection mode
const int DisconnectBot	= 0;
const int ShowBotMovement = 1;
const int ControlBotMovement = 2;

// all possible values for power on off
const int PowerOff = 0;
const int PowerOn	= 1;

// IDs of GLUI controls
const int InsertButtonID 	= 0;
const int OverwriteButtonID = 1;
const int DeleteButtonID 	= 2;
const int UpButtonID 		= 3;
const int DownButtonID 		= 4;
const int LoadButtonID 		= 5;
const int SaveButtonID 		= 6;
const int MergeButtonID 	= 7;

const int SimulateButtonID 		= 11;
const int StopButtonID 		= 13;
const int MoveButtonID 		= 14;
const int DeleteTrajButtonID= 15;
const int StepButtonID 		= 17;
const int CreateHanoiButtonID = 18;

const int RealTimeCheckBoxID = 16;

// controls
GLUI_List* trajectoryList = NULL;
GLUI_List* fileSelectorList = NULL;
GLUI_EditText* nodeNameControl 		= NULL;
GLUI_Spinner*  nodeTimeControl 		= NULL;
GLUI_Spinner*  nodeDurationControl 		= NULL;
GLUI_RadioGroup* interpolationTypeControl = NULL;
GLUI_RadioGroup* connectionToRealBotControl = NULL;

GLUI_StaticText* infoText 			= NULL;
GLUI_Panel* interactivePanel = NULL;
GLUI_FileBrowser* fileBrowser = NULL;
GLUI_Checkbox* botOnOffButton = NULL;
GLUI_RadioGroup* heartBeatButton = NULL;

GLUI_Checkbox* continouslyControl = NULL;
int continuouslyLiveVar = 0;

GLUI_EditText* startTimeLabel = NULL;
GLUI_EditText* endTimeLabel= NULL;
GLUI_EditText* startSpeedLabel= NULL;
GLUI_EditText* endSpeedLabel = NULL;
GLUI_EditText* durationLabel = NULL;

void trajectoryButtonCallback(int controlNo);


TrajectoryView::TrajectoryView() {
}

void TrajectoryView::display() {
	int idx = TrajectorySimulation::getInstance().getTrajectory().selected();
	if (idx >= 0)
		trajectoryList->set_current_item(idx);
}


void fillfileSelectorList() {
	int idx = fileSelectorList->get_current_item();
	string selectedFile;
	if ((idx >= 0) && (idx < (int)trajectoryFiles.size()))
		selectedFile = trajectoryFiles[idx];

	trajectoryFiles = readDirectory(".","trj");
	fileSelectorList->delete_all();
	for (unsigned i = 0;i<trajectoryFiles.size();i++) {
		fileSelectorList->add_item(i, trajectoryFiles[i].c_str());
		if (selectedFile == trajectoryFiles[i])
			fileSelectorList->set_current_item(i);
	}
}


void heartBeatCallback(int controlNo) {};

void powerOnOffCallback(int controlNo) {
	switch (powerOnOffLiveVar) {
	case PowerOff:
		TrajectorySimulation::getInstance().teardownBot();
		break;
	case PowerOn:
		TrajectorySimulation::getInstance().setupBot();
		if (!TrajectorySimulation::getInstance().botIsUpAndRunning())
			botOnOffButton->set_int_val(PowerOff);
		break;
	}
}

void fileSelectorListCallback(int controlNo) {
}

void TrajectoryView::loop() {
	// refresh file selector view every 5s
	static int timeExecution = -1;
	int time = (millis() / 1000) % 5;
	if ((time == 0) && (timeExecution != time)) {
		fillfileSelectorList();
		timeExecution = time;
	}
	if (timeExecution != time)
		timeExecution = -1;

	// check for bot heartbeat
	if (TrajectorySimulation::getInstance().heartBeatSendOp()) {
		heartBeatButton->set_int_val(0);
	} else {
		if (TrajectorySimulation::getInstance().heartBeatReceiveOp())
			heartBeatButton->set_int_val(1);
		else
			heartBeatButton->set_int_val(-1); // switch off when no heart beat
	}

}

void trajectoryListCallback(int controlNo) {
	// a new list item has been selected, use this node as pose
	int idx = trajectoryList->get_current_item();
	TrajectoryNode currentNode = TrajectorySimulation::getInstance().getTrajectory().select(idx);

	nodeTimeControl->set_int_val(currentNode.averageSpeedDef*1000.0);
	nodeDurationControl->set_int_val(currentNode.durationDef);
	interpolationTypeControl->set_int_val(currentNode.interpolationTypeDef);
	nodeNameControl->set_text(currentNode.name.c_str());

	// set detail data of trajectory node
	startTimeLabel->set_int_val(currentNode.time);
	endTimeLabel->set_int_val(currentNode.time + currentNode.duration);
	startSpeedLabel->set_int_val(currentNode.startSpeed*1000);
	endSpeedLabel->set_int_val(currentNode.endSpeed*1000);
	durationLabel->set_int_val(currentNode.duration);

	// set continously flag
	continouslyControl->set_int_val(currentNode.continouslyDef? 1:0);

	// set pose of bot to current node
	TrajectorySimulation::getInstance().setAngles(currentNode.pose.angles);
	TrajectorySimulation::getInstance().setPose(currentNode.pose);

	// if player is running, set it to the selected position
	TrajectorySimulation::getInstance().setPlayerPosition(currentNode.time);
}

void unsusedCallBack(int controlNo) {};

void connectionToRealBotCallback(int controlNo) {
	switch (connectionToRealBotLiveVar) {
	case DisconnectBot:
		TrajectorySimulation::getInstance().receiveFromRealBot(false);
		TrajectorySimulation::getInstance().sendToRealBot(false);
		break;
	case ShowBotMovement:
		if (TrajectorySimulation::getInstance().botIsUpAndRunning()) {
			TrajectorySimulation::getInstance().receiveFromRealBot(true);
			TrajectorySimulation::getInstance().sendToRealBot(false);
		} else
			connectionToRealBotControl->set_int_val(0); // bot not connected, switch back to "disconnect"
		break;
	case ControlBotMovement:
		if (TrajectorySimulation::getInstance().botIsUpAndRunning()) {
			TrajectorySimulation::getInstance().receiveFromRealBot(false);
			TrajectorySimulation::getInstance().sendToRealBot(true);
		} else
			connectionToRealBotControl->set_int_val(0); // bot not connected, switch back to "disconnect"

		break;
	}
}

void TrajectoryView::fillTrajectoryListControl() {
	trajectoryList->delete_all();

	vector<TrajectoryNode>& trajectory = TrajectorySimulation::getInstance().getTrajectory().getSupportNodes();

	for (unsigned int i = 0;i<trajectory.size();i++) {
		TrajectoryNode node =  trajectory[i];
		trajectoryList->add_item(i,node.getText().c_str());
	}
}

void trajectoryButtonCallback(int controlNo) {
	vector<TrajectoryNode>& trajectory = TrajectorySimulation::getInstance().getTrajectory().getSupportNodes();

	switch (controlNo) {
		case InsertButtonID: {
			// store current Pose, if different from the previous one
			TrajectoryNode node;
			node.pose = TrajectorySimulation::getInstance().getCurrentPose();
			node.pose.angles = TrajectorySimulation::getInstance().getCurrentAngles();

			node.name = trajectoryItemNameLiveVar;
			node.averageSpeedDef = float(trajectoryItemSpeedLiveVar)/1000.0;
			node.durationDef = trajectoryItemDurationLiveVar;
			node.interpolationTypeDef = (InterpolationType)interpolationTypeLiveVar;
			node.continouslyDef = continuouslyLiveVar;
			int idx = trajectoryList->get_current_item();

			vector<TrajectoryNode>::iterator trajListIter = trajectory.begin();

			if (trajectory.size() == 0) {
				trajectory.insert(trajectory.begin(), node);
				TrajectoryView::getInstance().fillTrajectoryListControl();
				trajectoryList->set_current_item(0);
			} else {
				int insertAtPos = idx+1;
				if ((insertAtPos == 0) || (trajectory[insertAtPos-1].pose.angles != node.pose.angles)) {
					trajectory.insert(trajectory.begin() + insertAtPos, node);
					TrajectoryView::getInstance().fillTrajectoryListControl();
					trajectoryList->set_current_item(insertAtPos);
				} else
					LOG(ERROR) << "pose to be inserted is identical to previous one";
			}

			break;
		}
		case OverwriteButtonID: {
			if (trajectory.size() == 0)
				trajectoryButtonCallback(InsertButtonID);
			else {
				// store current Pose
				TrajectoryNode node;
				node.pose = TrajectorySimulation::getInstance().getCurrentPose();
				node.pose.angles = TrajectorySimulation::getInstance().getCurrentAngles();
				node.name = trajectoryItemNameLiveVar;
				node.averageSpeedDef = float(trajectoryItemSpeedLiveVar)/1000.0;
				node.durationDef  = trajectoryItemDurationLiveVar;
				node.interpolationTypeDef = InterpolationType(interpolationTypeLiveVar);
				node.continouslyDef = continuouslyLiveVar;

				int idx = trajectoryList->get_current_item();
				int overwriteAt = (idx);
				trajectory[overwriteAt] = node;
				TrajectoryView::getInstance().fillTrajectoryListControl();
				trajectoryList->set_current_item(idx);
				trajectoryListCallback(0); // compilation changed details
			}
			break;
		}
		case DeleteButtonID: {
			if (trajectory.size() > 0) {
				int idx = trajectoryList->get_current_item();

				int deleteAt = idx;
				trajectory.erase(trajectory.begin() + deleteAt);
				TrajectoryView::getInstance().fillTrajectoryListControl();
				trajectoryList->set_current_item(idx);
			}
			break;
		}
		case UpButtonID: {
			if (trajectory.size() > 1) {
				int controlIdx= trajectoryList->get_current_item();
				int trajIdx = controlIdx;
				if ((unsigned)(trajIdx) > 0) {
					TrajectoryNode currNode = trajectory[trajIdx];
					TrajectoryNode prevNode= trajectory[trajIdx-1];
					trajectory[trajIdx] = prevNode;
					trajectory[trajIdx-1] = currNode;
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
				int trajIdx = (controlIdx);
				if (trajIdx < (int)trajectory.size()-1) {
					TrajectoryNode currNode = trajectory[trajIdx];
					TrajectoryNode dnNode= trajectory[trajIdx+1];
					trajectory[trajIdx] = dnNode;
					trajectory[trajIdx+1] = currNode;
					TrajectoryView::getInstance().fillTrajectoryListControl();
					trajectoryList->set_current_item(controlIdx+1);
				}
			}
			break;
		case SaveButtonID: {
			// find a free filename
			vector<TrajectoryNode>& trajectory = TrajectorySimulation::getInstance().getTrajectory().getSupportNodes();
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
				TrajectorySimulation::getInstance().getTrajectory().save(filename + ".trj");
				fillfileSelectorList();
			}
			break;
		}
		case LoadButtonID: {
			if (trajectoryFiles.size() > 0) {
				string filename;
				int idx = fileSelectorList->get_current_item();
				if (idx >= 0)
					filename = trajectoryFiles[idx];
				if (!filename.empty()) {
					TrajectorySimulation::getInstance().getTrajectory().load(filename);
					TrajectoryView::getInstance().fillTrajectoryListControl();
				}
			}
			break;
		}
		case MergeButtonID: {
			string filename;
			int idx = fileSelectorList->get_current_item();
			if (idx >= 0)
				filename = trajectoryFiles[idx];
			if (!filename.empty()) {
				TrajectorySimulation::getInstance().getTrajectory().merge(filename);
				TrajectoryView::getInstance().fillTrajectoryListControl();
			}
			break;
		}
		case DeleteTrajButtonID: {
			string filename;
			int idx = fileSelectorList->get_current_item();
			if (idx >= 0)
				filename = trajectoryFiles[idx];
			if (!filename.empty()) {
				remove(filename.c_str());
				fillfileSelectorList();
				TrajectoryView::getInstance().fillTrajectoryListControl();
			}
			break;
		}
		case CreateHanoiButtonID: {
			hanoi.solve(3);
			TrajectorySimulation::getInstance().getTrajectory().compile();
			TrajectoryView::getInstance().fillTrajectoryListControl();
			break;
		}

		default:
			break;
		}
	} // switch

	// compile trajectory ( timing and interpolation )
	TrajectorySimulation::getInstance().getTrajectory().compile();

	trajectoryListCallback(0); // compilation changed details

}

void trajectoryPlayerCallback (int controlNo) {
	switch (controlNo) {
	case StopButtonID: {
		TrajectorySimulation::getInstance().stopTrajectory();
		ExecutionInvoker::getInstance().stopTrajectory();

		break;
		}
	case SimulateButtonID: {
		connectionToRealBotCallback(DisconnectBot);
		TrajectorySimulation::getInstance().playTrajectory();
		break;
		}
	case StepButtonID: {
		if (TrajectorySimulation::getInstance().isOn())
			TrajectorySimulation::getInstance().step();
		else
			TrajectorySimulation::getInstance().stepTrajectory();
		break;
		}
	case MoveButtonID: {
		connectionToRealBotCallback(ShowBotMovement);
		Trajectory& trajectory = TrajectorySimulation::getInstance().getTrajectory();
		ExecutionInvoker::getInstance().runTrajectory(trajectory);
	}
	default:
		break;
	}
}

void TrajectoryView::create(GLUI *windowHandle, GLUI_Panel* pInteractivePanel) {
	interactivePanel = pInteractivePanel;
	GLUI_Panel* trajectoryPanel = new GLUI_Panel(interactivePanel,"trajectory panel", GLUI_PANEL_RAISED);
		new GLUI_StaticText(trajectoryPanel,"                          Trajectory Planning                           ");

	GLUI_Panel* trajectoryPlanningPanel = new GLUI_Panel(trajectoryPanel,"trajectory panel", GLUI_PANEL_NONE);
	trajectoryList = new GLUI_List(trajectoryPlanningPanel,"trajectory list", true, trajectoryListCallback);
	trajectoryList->set_h(115);
	trajectoryList->set_w(180);

	fillTrajectoryListControl();
    nodeNameControl = new GLUI_EditText( trajectoryPlanningPanel, "name", GLUI_EDITTEXT_TEXT, &trajectoryItemNameLiveVar, 0, unsusedCallBack );
	nodeTimeControl = new GLUI_Spinner( trajectoryPlanningPanel, "speed",GLUI_SPINNER_INT,  &trajectoryItemSpeedLiveVar, 0, unsusedCallBack);
	nodeTimeControl->set_int_limits(0,10000);
	nodeTimeControl->set_int_val(50);
	nodeDurationControl = new GLUI_Spinner( trajectoryPlanningPanel, "time",GLUI_SPINNER_INT,  &trajectoryItemDurationLiveVar, 0, unsusedCallBack);
	nodeDurationControl->set_int_limits(0,100000);
	nodeDurationControl->set_int_val(0);

	windowHandle->add_column_to_panel(trajectoryPlanningPanel, false);
	GLUI_Panel* trajectoryButtonPanel = new GLUI_Panel(trajectoryPlanningPanel,"", GLUI_PANEL_NONE);
	trajectoryButtonPanel->set_alignment(GLUI_ALIGN_LEFT);

	GLUI_Button* button = new GLUI_Button( trajectoryButtonPanel, "insert", InsertButtonID,trajectoryButtonCallback );
	button->set_w(70);
	button->set_alignment(GLUI_ALIGN_LEFT);

	button = new GLUI_Button( trajectoryButtonPanel, "overwrite", OverwriteButtonID,trajectoryButtonCallback );
	button->set_w(70);
	button->set_alignment(GLUI_ALIGN_LEFT);

	button = new GLUI_Button( trajectoryButtonPanel, "delete",DeleteButtonID,trajectoryButtonCallback  );
	button->set_w(70);
	button->set_alignment(GLUI_ALIGN_LEFT);

	button = new GLUI_Button( trajectoryButtonPanel, "up" ,UpButtonID,trajectoryButtonCallback );
	button->set_w(70);
	button->set_alignment(GLUI_ALIGN_LEFT);

	button = new GLUI_Button( trajectoryButtonPanel, "down",DownButtonID,trajectoryButtonCallback  );
	button->set_w(70);
	button->set_alignment(GLUI_ALIGN_LEFT);

	new GLUI_StaticText( trajectoryButtonPanel, "" );

	new GLUI_StaticText( trajectoryButtonPanel, "Interpolation" );
	interpolationTypeControl = new GLUI_RadioGroup( trajectoryButtonPanel,&interpolationTypeLiveVar,0, unsusedCallBack);
	interpolationTypeControl->set_alignment(GLUI_ALIGN_LEFT);
	new GLUI_RadioButton( interpolationTypeControl, "Linear" );
	new GLUI_RadioButton( interpolationTypeControl, "Bezier" );
	new GLUI_RadioButton( interpolationTypeControl, "Angles" );
	interpolationTypeControl->set_int_val(InterpolationType::POSE_CUBIC_BEZIER);


	windowHandle->add_column_to_panel(trajectoryPlanningPanel, false);
	GLUI_Panel* trajectoryDetailPanel = new GLUI_Panel(trajectoryPlanningPanel,"trajectory panel", GLUI_PANEL_NONE);
	trajectoryDetailPanel->set_w(100);
	startTimeLabel = new GLUI_EditText(trajectoryDetailPanel, "start [ms]");
	startTimeLabel->disable();
	startTimeLabel->set_alignment(GLUI_ALIGN_RIGHT);
	startTimeLabel->set_w(40);

	endTimeLabel= new GLUI_EditText(trajectoryDetailPanel, "end [ms]");
	endTimeLabel->disable();
	endTimeLabel->set_alignment(GLUI_ALIGN_RIGHT);
	endTimeLabel->set_w(40);

	startSpeedLabel= new GLUI_EditText(trajectoryDetailPanel, "v start [ms/s]");
	startSpeedLabel->disable();
	startSpeedLabel->set_alignment(GLUI_ALIGN_RIGHT);
	startSpeedLabel->set_w(40);

	endSpeedLabel = new GLUI_EditText(trajectoryDetailPanel, "v end [ms/s]");
	endSpeedLabel->disable();
	endSpeedLabel->set_alignment(GLUI_ALIGN_RIGHT);
	endSpeedLabel->set_w(40);

	durationLabel = new GLUI_EditText (trajectoryDetailPanel, "length [ms]");
	durationLabel->set_alignment(GLUI_ALIGN_RIGHT);
	durationLabel->set_w(40);
	durationLabel->disable();

	new GLUI_StaticText (trajectoryDetailPanel, "");
	new GLUI_StaticText (trajectoryDetailPanel, "");

	continouslyControl = new GLUI_Checkbox( trajectoryDetailPanel, "Continuous Movement",&continuouslyLiveVar,0 , unsusedCallBack);
	continouslyControl->set_alignment(GLUI_ALIGN_RIGHT);
	// trajectory planning
	GLUI_Panel* trajectoryHanoiPanel = new GLUI_Panel(interactivePanel,"trajectory&hanoi Panel", GLUI_PANEL_NONE);

	GLUI_Panel* trajectoryMgrPanel = new GLUI_Panel(trajectoryHanoiPanel,"trajectory move panel", GLUI_PANEL_RAISED);
	GLUI_StaticText* headline=new GLUI_StaticText(trajectoryMgrPanel,"                          trajectory management                      ");


	headline->set_alignment(GLUI_ALIGN_CENTER);
	GLUI_Panel* trajectoryMgrPanelPanel = new GLUI_Panel(trajectoryMgrPanel,"trajectory simulation  panel", GLUI_PANEL_NONE);
	GLUI_Panel* trajectoryMgrButtonPanel = new GLUI_Panel(trajectoryMgrPanelPanel,"trajectory simulation  panel", GLUI_PANEL_NONE);

	button = new GLUI_Button( trajectoryMgrButtonPanel, "Save" ,SaveButtonID,trajectoryButtonCallback );
	button->set_alignment(GLUI_ALIGN_CENTER);
	button->set_w(70);

	button = new GLUI_Button( trajectoryMgrButtonPanel, "Load",LoadButtonID,trajectoryButtonCallback  );
	button->set_alignment(GLUI_ALIGN_CENTER);
	button->set_w(70);
	windowHandle->add_column_to_panel(trajectoryMgrButtonPanel, false);

	button = new GLUI_Button( trajectoryMgrButtonPanel, "Merge",MergeButtonID,trajectoryButtonCallback  );
	button->set_alignment(GLUI_ALIGN_CENTER);
	button->set_w(70);
	button = new GLUI_Button( trajectoryMgrButtonPanel, "Delete",DeleteTrajButtonID,trajectoryButtonCallback  );
	button->set_alignment(GLUI_ALIGN_CENTER);
	button->set_w(70);

	windowHandle->add_column_to_panel(trajectoryMgrPanelPanel, false);
	fileSelectorList = new GLUI_List(trajectoryMgrPanelPanel,"trajectory list", true, fileSelectorListCallback);
	fileSelectorList->set_h(50);
	fileSelectorList->set_w(80);
	fillfileSelectorList();
	windowHandle->add_column_to_panel(trajectoryHanoiPanel, false);

	// hanoi panel
	GLUI_Panel* hanoiPanel = new GLUI_Panel(trajectoryHanoiPanel,"Hanoi Panel", GLUI_PANEL_RAISED);
	GLUI_StaticText* hanoiHeadline=new GLUI_StaticText(hanoiPanel,"Hanoi");
	button = new GLUI_Button( hanoiPanel, "create", CreateHanoiButtonID, trajectoryButtonCallback);

	// trajectory execution
	GLUI_Panel* trajectoryExecPanel = new GLUI_Panel(interactivePanel,"trajectory move panel", GLUI_PANEL_RAISED);
	headline=new GLUI_StaticText(trajectoryExecPanel,"                          bot control");
	GLUI_Panel* trajectoryExecConnectPanelPanel = new GLUI_Panel(trajectoryExecPanel,"trajectory execution  panel", GLUI_PANEL_NONE);

	button = new GLUI_Button( trajectoryExecConnectPanelPanel, "simulate", SimulateButtonID, trajectoryPlayerCallback);
	button->set_w(70);

	button = new GLUI_Button( trajectoryExecConnectPanelPanel, "STOP", StopButtonID, trajectoryPlayerCallback);
	button->set_w(70);
	windowHandle->add_column_to_panel(trajectoryExecConnectPanelPanel, false);

	button = new GLUI_Button( trajectoryExecConnectPanelPanel, "step", StepButtonID, trajectoryPlayerCallback);
	button->set_w(70);

	button = new GLUI_Button( trajectoryExecConnectPanelPanel, "move", MoveButtonID, trajectoryPlayerCallback);
	button->set_w(70);
	windowHandle->add_column_to_panel(trajectoryExecConnectPanelPanel, false);

	connectionToRealBotControl = new GLUI_RadioGroup( trajectoryExecConnectPanelPanel,&connectionToRealBotLiveVar,0, connectionToRealBotCallback);
	new GLUI_RadioButton( connectionToRealBotControl, "disconnect" );
	new GLUI_RadioButton( connectionToRealBotControl, "show bot" );
	new GLUI_RadioButton( connectionToRealBotControl, "control bot" );

	windowHandle->add_column_to_panel(trajectoryExecConnectPanelPanel, false);
	botOnOffButton = new GLUI_Checkbox( trajectoryExecConnectPanelPanel, "ON",&powerOnOffLiveVar,0 , powerOnOffCallback);
	static int unusedLiveVar;
	heartBeatButton = new GLUI_RadioGroup( trajectoryExecConnectPanelPanel,&unusedLiveVar, 0, heartBeatCallback);
	heartBeatButton->set_alignment(GLUI_ALIGN_LEFT);
	heartBeatButton->disable();
	new GLUI_RadioButton( heartBeatButton, "tx" );
	new GLUI_RadioButton( heartBeatButton, "rx");
}
