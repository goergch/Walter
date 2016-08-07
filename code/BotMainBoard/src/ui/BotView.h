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
	void printSubWindowTitle(std::string text, const GLfloat *titleColor );
	void drawCoordSystem(bool withRaster, const GLfloat *rasterColor, const GLfloat *axisColor  );
	void paintBot(JointAngleType angles, const GLfloat *subWindowColor);
};

#endif /* UI_BOTVIEW_H_ */
