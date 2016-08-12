/*
 * TrajectoryView.h
 *
 *  Created on: 12.08.2016
 *      Author: SuperJochenAlt
 */

#ifndef UI_TRAJECTORYVIEW_H_
#define UI_TRAJECTORYVIEW_H_

class TrajectoryView {
public:
	TrajectoryView();

	void create();
	void display();
	void reshape(int x,int y, int w, int h);

private:
	int windowHandle;
};

#endif /* UI_TRAJECTORYVIEW_H_ */
