/*
DeckLinkDisplay.cpp

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




#include "DeckLinkDisplay.h"


DeckLinkDisplay::DeckLinkDisplay(QObject *parent)
{
    _running = false;
    _head = false;
    _decklink = new DeckLinkCap();
}


DeckLinkDisplay::~DeckLinkDisplay()
{
    if (_running == true) {
        this->stop();
    }
}

bool DeckLinkDisplay::start() {
    if (_decklink->IsInitialized() && _running != true) {

        _decklink_output_left = _decklink->InitOutputAtPort(VIDEO_OUT_L);
        _decklink_output_right = _decklink->InitOutputAtPort(VIDEO_OUT_R);
        _decklink_output_left->Start();
        _decklink_output_right->Start();
        _running = true;
    } else {

        return false;
    }
    return true;
}

bool DeckLinkDisplay::stop() {
    if (_running == true) {
        _decklink_output_left->Stop();
        _decklink_output_right->Stop();
        _running = false;
    }
    return true;
}


bool DeckLinkDisplay::isRunning() {
    return _running;
}


void DeckLinkDisplay::displayHeadCalibration(bool display) { // display is true during head calibration
    _head = display;


}

void DeckLinkDisplay::displayImage(float x, float y, float size, QColor color, bool is_head) {

    if (_head && !is_head) {
        return;
    }

    qDebug() << "displayim" << _running << "(" << x << "," << y << ")" << size;

    //cv::imwrite("C:/Users/Irene/Pictures/decklinkout.jpg",final_im);
    if (_running == true) {
        /* cv::Mat cv_im(newim.height(),newim.width(),CV_8UC4,(void *)newim.constBits(),newim.bytesPerLine());
        //cv::Mat cv_im = cv::Mat::zeros(cv::Size(newim.width(),newim.height()),CV_8UC4);
        cv::Mat flip_im,final_im;

        // cv::cvtColor(cv_im,final_im,CV_RGBA2RGBA);
        //cv::imwrite("C:/Users/Irene/Pictures/decklinkout.jpg",cv_im);
        cv::flip(cv_im,flip_im,0);

        cv::resize(flip_im,flip_im,cv::Size(1920,1080));
        flip_im.copyTo(final_im);*/

        cv::Mat final_im = cv::Mat(cv::Size(1920,1080),CV_8UC3,cv::Scalar::all(0.8));
        final_im = final_im * 200;

        cv::Scalar cvcol;
        if (color == Qt::red) {
            cvcol = cv::Scalar(0,0,255,255);
        } else {
            cvcol = cv::Scalar(0,255,0,255);
        }
        cv::circle(final_im,cv::Point(float(x)*1920,float(1-y)*1080),size*2,cvcol,-1);

        cv::circle(final_im,cv::Point(float(x)*1920,float(1-y)*1080),10,cvcol/2,-1);

        cv::cvtColor(final_im,final_im,cv::COLOR_BGR2BGRA);
        int imsize = final_im.rows * final_im.cols * final_im.channels();
        _decklink_output_left->UpdateBuffers(final_im.data,imsize);
        _decklink_output_right->UpdateBuffers(final_im.data, imsize);

    }
}

     
