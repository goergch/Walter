/*
 * uiconfig.h
 *
 * Author: JochenAlt
 */

#ifndef UI_UICONFIG_H_
#define UI_UICONFIG_H_

// constants used in the UI
const float ViewEyeDistance 		= 1500.0f;						// distance of the eye to the bot
const float ViewBotHeight 			= 800.0f;						// height of the bot to be viewed
const int pearlChainDistance_ms		= (BotTrajectorySampleRate);	// trajectories are display with pearls in a timing distance

// window size
const int WindowGap=10;							// gap between subwindows
const int InteractiveWindowWidth=588;			// initial width of the console window

// used colors
static const GLfloat glMainWindowColor[]         = {1.0,1.0,1.0};
static const GLfloat glBlackColor[] 			= { 0.0f, 0.0f, 0.0f };
static const GLfloat glWhiteColor[] 			= { 1.0f, 1.0f, 1.0f };

static const GLfloat glCoordSystemAreaColor3v[]	= { 0.97f, 0.97f, 0.97f };
static const GLfloat glCoordSystemColor3v[] 	= { 0.40f, 0.40f, 0.6f };
static const GLfloat glRasterColor3v[] 			= { .85f, .9f, 0.9f };
static const GLfloat glWindowTitleColor[] 		= { 1.0f, 1.0f, 1.0f };
static const GLfloat glTCPColor3v[] 			= { 0.23f, 0.62f, 0.94f };
static const GLfloat startPearlColor[] 			= { 0.23f, 1.0f, 0.24f };
static const GLfloat endPearlColor[] 			= { 0.90f, 0.2f, 0.2f };
static const GLfloat midPearlColor[] 			= { 0.80f, 1.00f, 0.80f };

static const GLfloat glBotaccentColor[] 		= {122.0/255.0, 118.0/255.0, 105.0/255.0 };            // Moosgrau
static const GLfloat glBotJointColor3DView[] 			= { 122.0/255.0, 106.0/255.0, 142.0/255.0 };   // fehgrau
static const GLfloat glBotArmColor3DView[] 				= { 108.0/255.0, 123.0/255.0, 89.0/255.0 };    // reseda grün
static const GLfloat glSubWindowColor3DView[] 			= { 219.0/256.0, 238.0/256.0, 244.0/256.0};

// static const GLfloat glBotJointColor3DView[] 			= { 0.5f, 0.6f, 0.6f };
// static const GLfloat glBotArmColor3DView[] 				= { 0.9f, 0.3f, 0.2f };
// static const GLfloat glSubWindowColor3DView[] 			= { 219.0/256.0, 238.0/256.0, 244.0/256.0};

#endif /* UI_UICONFIG_H_ */
