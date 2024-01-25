/*
DeckLinkDisplay.h

Author(s): Irene Tong, Maxwell Li
Created on: August 1, 2018

(c) Copyright 2018 University of British Columbia

--- begin license - do not edit ---

    This file is part of CGaze UI. 
   
    CGaze UI is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Cgaze UI is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Cgaze UI.  If not, see <https://www.gnu.org/licenses/>.
--- end license ---
*/

#ifndef DECKLINKDISPLAY_H
#define DECKLINKDISPLAY_H

// A stereo image through the DeckLink Duo 2 card.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "DeckLinkCap.h"


#include <QObject>
#include <QImage>
#include <QDebug>
enum DeckLinkDVPort {
    VIDEO_OUT_L = 2,
    VIDEO_OUT_R = 4,
    VIDEO_OUT_US = 1,
    VIDEO_IN_US = 3,
};

class DeckLinkDisplay : public QObject
{
    Q_OBJECT

public:
    DeckLinkDisplay(QObject *parent = 0);
    ~DeckLinkDisplay();
    bool start();
    bool isRunning();
    bool stop();
    void displayHeadCalibration(bool display);
public slots:
    void displayImage(float x, float y, float size, QColor color, bool is_head);

private:
    DeckLinkOutputDevice* _decklink_output_left;
    DeckLinkOutputDevice* _decklink_output_right;
    DeckLinkCap*			  _decklink;

    bool _connected;
    bool _running;
    bool _head;
};

#endif // DECKLINKDISPLAY_H
