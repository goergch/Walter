/*
 * BotViewController.h
 *
 *  Created on: 07.08.2016
 *      Author: SuperJochenAlt
 */

#ifndef UI_BOTVIEW_H_
#define UI_BOTVIEW_H_

class BotView {
public:
	BotView()  {};
	void setLights();
	void printSubWindowTitle(std::string text );
	void drawCoordSystem(bool withRaster );
	void paintBot(JointAngleType angles);
};

#endif /* UI_BOTVIEW_H_ */
