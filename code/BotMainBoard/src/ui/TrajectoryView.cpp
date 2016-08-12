/*
 * TrajectoryView.cpp
 *
 *  Created on: 12.08.2016
 *      Author: SuperJochenAlt
 */

#include <ui/TrajectoryView.h>


std::string trajectoryItemNameLiveVar;
int trajectoryItemDurationLiveVar;
int trajectorySmoothLiveVar;

TrajectoryView::TrajectoryView() {
}

void trajectoryListCallback(int controlNo) {

}

void trajectoryDurationCallback(int controlNo) {

}

void trajectoryNameCallback(int controlNo) {

}

void trajectorySmoothCallback(int controlNo) {

}

void trajectoryButtonCallback(int controlNo) {

}



void TrajectoryView::create(GLUI *windowHandle, GLUI_Panel* interactivePanel) {
	GLUI_Panel* trajectoryPanel = new GLUI_Panel(interactivePanel,"trajectory panel", GLUI_PANEL_RAISED);
		new GLUI_StaticText(trajectoryPanel,"                          trajectory planning                           ");

		GLUI_Panel* trajectoryPlanningPanel = new GLUI_Panel(trajectoryPanel,"trajectory panel", GLUI_PANEL_NONE);

		GLUI_List* trajectoryList = new GLUI_List(trajectoryPlanningPanel,"trajectory list", true, trajectoryListCallback);
		trajectoryList->add_item( 0, "<nil>" );
		trajectoryList->set_h(90);
		new GLUI_EditText( trajectoryPlanningPanel, "name", GLUI_LIVE_STRING, &trajectoryItemNameLiveVar, 0, trajectoryNameCallback );
		new GLUI_EditText( trajectoryPlanningPanel, "time[s]",GLUI_EDITTEXT_INT,  &trajectoryItemDurationLiveVar, 0, trajectoryDurationCallback);

		windowHandle->add_column_to_panel(trajectoryPlanningPanel, false);
		GLUI_Panel* trajectoryButtonPanel = new GLUI_Panel(trajectoryPlanningPanel,"trajectory  button panel", GLUI_PANEL_NONE);
		new GLUI_Button( trajectoryButtonPanel, "save", 0,trajectoryButtonCallback );
		new GLUI_Button( trajectoryButtonPanel, "delete",1,trajectoryButtonCallback  );
		new GLUI_Button( trajectoryButtonPanel, "up" ,2,trajectoryButtonCallback );
		new GLUI_Button( trajectoryButtonPanel, "down",3,trajectoryButtonCallback  );
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
