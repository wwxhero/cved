//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2016 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: RoadTraveler.h,v 1.2 2018/12/07 21:14:18 IOWA\dheitbri Exp $
//
// Author(s):	David Heitbrink
// Date:		October, 1999
//
// Description:	Header CRoadTraveler
//
//////////////////////////////////////////////////////////////////////////////
#include "transmat.h"
#include "cved.h"
#include "roadpos.h"
#include <utility>      // std::pair 

#ifndef __ROADTRAVELER_H_
#define __ROADTRAVELER_H
#pragma once
namespace CVED {

    typedef struct {

        struct {
            std::vector<int> sameForward;
            std::vector<int> sameBackward;
            std::vector<int> leftForward;
            std::vector<int> leftBackward;
            std::vector<int> rightForward;
            std::vector<int> rightBackward;
        } ids;

        struct {
            std::vector<float> sameForward;
            std::vector<float> sameBackward;
            std::vector<float> leftForward;
            std::vector<float> leftBackward;
            std::vector<float> rightForward;
            std::vector<float> rightBackward;
        } dist;
    }TTrafficCloudData;

    class CRoadTraveler
    {
    public:
        CRoadTraveler(CVED::CRoadPos& pos, const CVED::CCved&, CVED::CPath& = CVED::CPath());
        bool Travel(float&, std::vector<int>& objects, std::vector<float>& dists, int ignoreId = -1);
        TTrafficCloudData GetTrafficCloudInformation(
            float dist, int id);

        ~CRoadTraveler(void);
    private:
        bool TravelPriv(float&, float&, CVED::CRoadPos& pos, std::vector<int>& objects, std::vector<float>& dists, int ignoreId = -1);
        bool TravelPrivRoad(float&, float&, CVED::CRoadPos& pos, std::vector<int>& objects, std::vector<float>& dists, int ignoreId = -1);
        bool TravelPrivCrdr(float&, float&, CVED::CRoadPos& pos, std::vector<int>& objects, std::vector<float>& dists, int ignoreId = -1);
        bool TravelPrivRoadNeg(float&, float&, CVED::CRoadPos& pos, std::vector<int>& objects, std::vector<float>& dists, int ignoreId = -1);
        bool TravelPrivCrdrNeg(float&, float&, CVED::CRoadPos& pos, std::vector<int>& objects, std::vector<float>& dists, int ignoreId = -1);
        CVED::CRoadPos m_pos;
        CVED::CPath m_path;
        const CVED::CCved& m_cCved;
    };

}
#endif
