#include "cvedpub.h"
#include "cvedstrc.h"	// private CVED data structs
#include "RoadTraveler.h"
using namespace CVED;
using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn CRoadTraveler::CRoadTraveler(CVED::CRoadPos& pos,const CVED::CCved& cved)
///
/// \brief  Constructor.
///
/// \author Dheitbri
/// \date   5/31/2017
///
/// \param [in,out] pos     The position.
/// \param          cved    The cved.
////////////////////////////////////////////////////////////////////////////////////////////////////
CVED::CRoadTraveler::CRoadTraveler(CVED::CRoadPos& pos,const CVED::CCved& cved, CPath&):m_pos(pos),m_cCved(cved),m_path(cved)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn CRoadTraveler::~CRoadTraveler(void)
///
/// \brief  Destructor.
///
/// \author Dheitbri
/// \date   5/31/2017
////////////////////////////////////////////////////////////////////////////////////////////////////

CVED::CRoadTraveler::~CRoadTraveler(void)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn bool CRoadTraveler::Travel(float& dist, std::vector<int>& objects)
///
/// \brief  Travels.
///
/// \author Dheitbri
/// \date   5/31/2017
///
/// \param [in,out] dist    The distance.
/// \param [out]    objects The objects.
///
/// \return True if it succeeds, false if it fails.
////////////////////////////////////////////////////////////////////////////////////////////////////

bool 
CVED::CRoadTraveler::Travel(float& dist, std::vector<int>& objects, std::vector<float>& dists, int ignoreId){
    if (!m_pos.IsValid())
        return false;
    objects.clear();
    dists.clear();
    float cumulativeDist = 0;
    return TravelPriv(dist,cumulativeDist, m_pos, objects,dists,ignoreId);
}
bool
FindDestCrdr(CVED::CLane& pos, CVED::CIntrsctn &intr, int &id){
    TCrdrVec vec;
    intr.GetCrdrsStartingFrom(pos, vec);
    if (vec.size() == 0) return false;
    for (auto itr = vec.begin(); itr != vec.end(); ++itr) {
        if (itr->GetCrdrDirection() == CVED::CCrdr::eSTRAIGHT) {
            id = itr->GetRelativeId();
            return true;
       }
    }
    id = vec.back().GetRelativeId();
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn bool FindSrcCrdr(CVED::CLane& pos, CVED::CIntrsctn &intr, int &id)
///
/// \brief  Searches for straight source crdr.
///
/// \author Dheitbri
/// \date   6/19/2017
///
/// \param [in] pos     The position.
/// \param [in] intr    The intersection.
/// \param [out] id     Relative Crdr ID.
///
/// \return True if it succeeds, false if it fails.
////////////////////////////////////////////////////////////////////////////////////////////////////

bool
FindSrcCrdr(CVED::CLane& pos, CVED::CIntrsctn &intr, int &id){
    TCrdrVec vec;
    intr.GetCrdrsLeadingTo(pos, vec);
    if (vec.size() == 0) return false;
    for (auto itr = vec.begin(); itr != vec.end(); ++itr) {
        if (itr->GetCrdrDirection() == CVED::CCrdr::eSTRAIGHT) {
            id = itr->GetRelativeId();
            return true;
        }
    }
    if (vec.size() ==0) return false;
    id = vec.back().GetRelativeId();
	return true; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn bool CRoadTraveler::TravelPrivRoad(float& dist, CVED::CRoadPos& pos, std::vector<int>& objects)
///
/// \brief  Travel priv road.
///
/// \author Dheitbri
/// \date   5/31/2017
///
/// \param [in,out] dist    The distance.
/// \param [in,out] pos     The position.
/// \param [in,out] objects The objects.
///
/// \return True if it succeeds, false if it fails.
////////////////////////////////////////////////////////////////////////////////////////////////////

bool 
CVED::CRoadTraveler::TravelPrivRoad(float& dist,float& cumulativeDist, CVED::CRoadPos& pos, std::vector<int>& objects, std::vector<float>& dists, int ignoreId) {
    CVED::CLane lane = pos.GetLane();
    int id = lane.GetId();
    vector<CCved::TObjWithDist> res;
    float targetDist = dist;
    float deltaD = 0;
    m_cCved.GetAllDynObjsOnLane(lane, res, eCV_VEHICLE);
    float sign = 1.0;
    if (lane.GetDirection() == eNEG) {
        sign = -1;
    }
    double myDist = pos.GetDistance();
	double laneDistance = lane.GetRoad().GetCubicLength();
	double distMax = pos.GetDistance() + dist*sign;
    auto dir = lane.GetDirection();
    if (dir == eNEG) {
        for (auto itr = res.rbegin(); itr != res.rend(); ++itr) {
            if (itr->objId == ignoreId) continue;
			double dist = myDist - itr->dist ;
            if (itr->dist > distMax && dist > 0) {
                objects.push_back(itr->objId);
                dists.push_back(fabs(dist) + cumulativeDist);
            }
        }
    }
    else {
        for (auto itr = res.begin(); itr != res.end(); ++itr) {
            if (itr->objId == ignoreId) continue;
			double dist = itr->dist - myDist;
            if ((dist)*sign < fabs(distMax)*sign && dist*sign > 0) {
                objects.push_back(itr->objId);
                dists.push_back(dist + cumulativeDist);
            }
        }
    }
    
    if (dir == eNEG) {
        deltaD= myDist;
        myDist = myDist - dist;
        if (myDist < 0) {
           dist = myDist*-1;
        } else{
            return true;
        }
    }else{
        deltaD += laneDistance-myDist;
        myDist = myDist + dist;
        float laneDist= lane.GetRoad().GetLinearLength();
        if (myDist >laneDist) {
            dist = laneDist;
        }else{
            return true;
        }
    } 
    
    auto intr =lane.GetNextIntrsctn();
    if (!intr.IsValid()) return false;
    int crdrId = -1;
    if (m_path.IsValid()){
        if (!m_path.GetCrdrFromIntrscn(intr.GetId(), crdrId, &pos, lane.GetId())) {
            FindDestCrdr(lane,intr, crdrId);
        }
    }else{
        FindDestCrdr(lane, intr, crdrId);
    }
    if (crdrId < 0)
        return false;
    CVED::CRoadPos newpos(intr, crdrId);
    targetDist -= deltaD;
    cumulativeDist += deltaD;
    return TravelPrivCrdr(targetDist, cumulativeDist, newpos, objects,dists);

    //float length = lane.GetRoad().GetCubicLength();
    //if (myDist < 0) {
    //    if (dir == eNEG) {
    //        auto intr = lane.GetPrevIntrsctn();
    //        if (!intr.IsValid()) {
    //            return true;
    //        }
    //        else {
    //            CVED::CCrdr crdr;
    //            if (dist < 0)
    //                FindDestCrdr(lane, intr, crdr);
    //            else
    //                FindSrcCrdr(lane, intr, crdr);
    //            if (crdr.IsValid()) {
    //                CRoadPos newPos(intr, crdr.GetId(), crdr.GetLength());
    //                dist = length - myDist;
    //                TravelPriv(dist, newPos, objects);
    //            }
    //            return true;
    //        }
    //    }
    //    else if (dir == eNEG && dist > 0) {

    //    }
    //}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn bool CRoadTraveler::TravelPrivCrdr(float& dist, CVED::CRoadPos& pos, std::vector<int>& objects)
///
/// \brief  Travel priv crdr.
///
/// \author Dheitbri
/// \date   5/31/2017
///
/// \param [in,out] dist    The distance.
/// \param [in,out] pos     The position.
/// \param [in,out] objects The objects.
///
/// \return True if it succeeds, false if it fails.
////////////////////////////////////////////////////////////////////////////////////////////////////

bool 
CVED::CRoadTraveler::TravelPrivCrdr(float& dist,float& cumulativeDist, CVED::CRoadPos& pos, std::vector<int>& objects, std::vector<float>& dists, int ignoreId) {
    CVED::CCrdr crdr = pos.GetCorridor();
    int id = crdr.GetId();
    vector<CCved::TObjWithDist> res;
    m_cCved.GetAllDynObjsOnCrdr(crdr.GetIntrsctnId(),crdr.GetRelativeId(), res, eCV_VEHICLE);
    double myDist = pos.GetDistance();
    float initDist = dist;
    float distMax = myDist + dist;
    for (auto itr = res.begin(); itr != res.end(); ++itr) {
        if (itr->objId == ignoreId) continue;
        if (myDist < itr->dist) {
            objects.push_back(itr->objId);
            dists.push_back(fabs(itr->dist - myDist) + cumulativeDist);
        }
    }
    dist = dist - crdr.GetLength();
    if (dist<0) return true;
    auto lane = crdr.GetDstntnLn();
    auto road = lane.GetRoad();
    float roaddist = 0;
    if (lane.GetDirection() == eNEG)
        roaddist = road.GetLinearLength();
    CRoadPos posOut(road, lane, roaddist);
    float deltaD = crdr.GetLength() - myDist;
    cumulativeDist += deltaD;
    dist = initDist - deltaD;
    return TravelPrivRoad(dist,cumulativeDist, posOut, objects,dists);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn bool CRoadTraveler::TravelPrivRoad(float& dist, CVED::CRoadPos& pos, std::vector<int>& objects)
/// 
/// \brief  Travel priv road.
/// Private method for traveling the road in the positive direction
/// \author Dheitbri
/// \date   5/31/2017
///
/// \param [in,out] dist    The distance.
/// \param [in,out] pos     The position.
/// \param [in,out] objects The objects Ids.
///
/// \return True if it succeeds, false if it fails.
////////////////////////////////////////////////////////////////////////////////////////////////////

bool
CVED::CRoadTraveler::TravelPrivRoadNeg(float& dist,float& cumulativeDist, CVED::CRoadPos& pos, std::vector<int>& objects, std::vector<float>& dists, int ignoreId) {
    dist = fabs(dist);
    float initDist = dist;
    float deltaD = 0;
    CVED::CLane lane = pos.GetLane();
	double roadDist = lane.GetRoad().GetCubicLength();
    int id = lane.GetId();
    vector<CCved::TObjWithDist> res;
    m_cCved.GetAllDynObjsOnLane(lane, res, eCV_VEHICLE);
    float sign = 1.0;
    auto dir = lane.GetDirection();
    if (dir == eNEG) sign = -1.0;
    double myDist = pos.GetDistance();
    float distMin = pos.GetDistance() - dist*sign;
    if (distMin<0) distMin = 0;
    if (dir == ePOS) {
        for (auto itr = res.rbegin(); itr != res.rend(); ++itr) {
            if (itr->objId == ignoreId) continue;
            if ((itr->dist - myDist)*sign < 0 && itr->dist*sign > distMin *sign) {
                objects.push_back(itr->objId);
                dists.push_back(cumulativeDist - fabs(itr->dist - myDist));
            }
        }
    } else {
        for (auto itr = res.begin(); itr != res.end(); ++itr) {
            if (itr->objId == ignoreId) continue;
            if ((itr->dist - myDist)*sign < 0 && itr->dist*sign > distMin *sign) {
                objects.push_back(itr->objId);
                dists.push_back(cumulativeDist - fabs(itr->dist - myDist));
            }
        }
    }
    auto roadLenght = lane.GetRoad().GetCubicLength();
    if (dir == eNEG) {
        deltaD = roadDist - myDist;
        cumulativeDist -= deltaD;
        myDist = myDist + dist;
        if (roadLenght < myDist) {
            dist = myDist - dist;
        }else {
            return true;
        }
    }
    else {
        deltaD = myDist;
        cumulativeDist -= deltaD;
        myDist = myDist - dist;
        if (myDist < 0) {
            dist = fabs(myDist);
        }
        else {
            return true;
        }
    }
    auto intr = lane.GetPrevIntrsctn();
    if (!intr.IsValid()) return false;
    int crdrId = -1;
    if (m_path.IsValid()) {
        if (!m_path.GetCrdrFromIntrscn(intr.GetId(), crdrId, &pos, lane.GetId())) {
            FindSrcCrdr(lane, intr, crdrId);
        }
    }
    else {
        FindSrcCrdr(lane, intr, crdrId);
    }
    if (crdrId < 0)
        return false;
    CVED::CRoadPos newpos(intr, crdrId);
    auto crdr = newpos.GetCorridor();
    newpos.SetDistance(crdr.GetLength());
    float targetDist = 0 - (fabs(initDist) - fabs(deltaD));
    return TravelPrivCrdrNeg(targetDist,cumulativeDist, newpos, objects,dists);

    //float length = lane.GetRoad().GetCubicLength();
    //if (myDist < 0) {
    //    if (dir == eNEG) {
    //        auto intr = lane.GetPrevIntrsctn();
    //        if (!intr.IsValid()) {
    //            return true;
    //        }
    //        else {
    //            CVED::CCrdr crdr;
    //            if (dist < 0)
    //                FindDestCrdr(lane, intr, crdr);
    //            else
    //                FindSrcCrdr(lane, intr, crdr);
    //            if (crdr.IsValid()) {
    //                CRoadPos newPos(intr, crdr.GetId(), crdr.GetLength());
    //                dist = length - myDist;
    //                TravelPriv(dist, newPos, objects);
    //            }
    //            return true;
    //        }
    //    }
    //    else if (dir == eNEG && dist > 0) {

    //    }
    //}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn bool CRoadTraveler::TravelPrivCrdr(float& dist, CVED::CRoadPos& pos, std::vector<int>& objects)
///
/// \brief  Travel priv crdr. neg dir
/// Travels the road in the negitive direction
/// \author Dheitbri
/// \date   5/31/2017
///
/// \param [in,out] dist    The distance.
/// \param [in,out] pos     The position.
/// \param [in,out] objects The objects.
///
/// \return True if it succeeds, false if it fails.
////////////////////////////////////////////////////////////////////////////////////////////////////
bool
CVED::CRoadTraveler::TravelPrivCrdrNeg(float& dist,float& cumulativeDist, CVED::CRoadPos& pos, std::vector<int>& objects, std::vector<float>& dists, int ignoreId) {
    CVED::CCrdr crdr = pos.GetCorridor();
    int id = crdr.GetId();
    vector<CCved::TObjWithDist> res;
    m_cCved.GetAllDynObjsOnCrdr(crdr.GetIntrsctnId(), crdr.GetRelativeId(), res, eCV_VEHICLE);
    double myDist = pos.GetDistance();
    float distMin = myDist - dist;
    if (distMin<0) distMin = 0;
    for (auto itr = res.begin(); itr != res.end(); ++itr) {
        if (itr->objId == ignoreId) continue;
        float targdist = myDist - itr->dist;
        if ((targdist) > 0 && targdist < fabs(dist)) {
            objects.push_back(itr->objId);
            dists.push_back(cumulativeDist - targdist);
        }
    }
    if (myDist>fabs(dist)) return true;
    dist = fabs(dist);
    auto lane = crdr.GetSrcLn();
    auto road = lane.GetRoad(); 
    float roaddist = 0;
    if (lane.GetDirection() == ePOS)
        roaddist = road.GetLinearLength();
    CRoadPos posOut(road, lane, roaddist);
    cumulativeDist -= myDist;
    dist  = 0 - (dist - myDist);
    return TravelPrivRoadNeg(dist, cumulativeDist, posOut, objects,dists);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn bool CRoadTraveler::TravelPriv(float dist, CVED::CRoadPos& pos, std::vector<int>& objects)
///
/// \brief  Travel Road in Positive Direction
///
/// \author Dheitbri
/// \date   5/31/2017
///
/// \param          dist    The distance.
/// \param [in,out] pos     The position.
/// \param [in,out] objects The objects.
///
/// \return True if it succeeds, false if it fails.
////////////////////////////////////////////////////////////////////////////////////////////////////

bool 
CVED::CRoadTraveler::TravelPriv(float& dist,float& cumulativeDist, CVED::CRoadPos& pos, std::vector<int>& objects, std::vector<float>& dists, int ignoreId){
	if (dist > 0) {
        if (pos.IsRoad())
            return TravelPrivRoad(dist,cumulativeDist,pos,objects,dists,ignoreId);
        else
            return TravelPrivCrdr(dist,cumulativeDist,pos,objects,dists,ignoreId);
    }
    else {
        if (pos.IsRoad())
            return TravelPrivRoadNeg(dist,cumulativeDist, pos, objects,dists,ignoreId);
        else
            return TravelPrivCrdrNeg(dist,cumulativeDist, pos, objects,dists,ignoreId);
    }
}

TTrafficCloudData  CVED::CRoadTraveler::GetTrafficCloudInformation(
    float dist, int id) {
    TTrafficCloudData res;
    if (!m_pos.IsValid())
        return res;
    

    float fdist = dist;
    CRoadPos rightPos,leftPos,samePos;
    /////////////////////////////////////
    //Same Lane
    ///////////////////////////////////// 
    {
        samePos = m_pos;
        CRoadTraveler rt(samePos, m_cCved, m_path);
        float negDist = -fdist;
        rt.Travel(negDist, res.ids.sameBackward,res.dist.sameBackward,id);
        fdist = dist;
        CRoadTraveler rtf(samePos, m_cCved, m_path);
        
        rtf.Travel(fdist, res.ids.sameForward,res.dist.sameForward,id);
    }
    /////////////////////////////////////
    //Right Lane
    ///////////////////////////////////// 
    rightPos = m_pos;
    if (rightPos.IsRoad()){
        if (rightPos.ChangeLaneRight()){
            CRoadTraveler rt(rightPos, m_cCved, m_path);
            fdist = -dist;
            rt.Travel(fdist, res.ids.rightBackward, res.dist.rightBackward,id);
            fdist = dist;
            rt.Travel(fdist, res.ids.rightForward,res.dist.rightForward,id);
        }
    } else {
        auto crdr = rightPos.GetCorridor();
        auto lane = crdr.GetSrcLn();
        TCrdrVec srcCrds,destCrds;
        if (lane.IsValid()){
            CRoadPos rp(lane.GetRoad(),lane);
            if (rp.ChangeLaneRight()){
                auto intr = rightPos.GetIntrsctn();
                auto rightLane = rp.GetLane();
                intr.GetCrdrsStartingFrom(rightLane, srcCrds);
            }
            auto lanedest = crdr.GetDstntnLn();
            CRoadPos rp2(lanedest.GetRoad(), lane);
            if (rp2.IsValid() && rp2.ChangeLaneRight()) {
                auto intr = rightPos.GetIntrsctn();
                auto rightLane = rp2.GetLane();
                intr.GetCrdrsLeadingTo(rightLane, destCrds);
            }
        }
        CCrdr target;
        for (auto itr = srcCrds.begin(); itr != srcCrds.end(); ++itr) {
            bool foundTarget = false;

            for (auto jtr = destCrds.begin(); jtr != destCrds.end(); ++jtr) {
                if (itr->GetId() == jtr->GetId()) {
                    target = *itr;
                    foundTarget = true;
                    break;
                }
                if (foundTarget) break;
            }
        }
        if (target.IsValid()) {
            float tdist = m_pos.GetDistance();
            if (tdist > target.GetLength()) {
                tdist = target.GetLength();
            }
            CRoadPos pos(target.GetIntrsctn(),target, tdist);
            CRoadTraveler rt(pos, m_cCved, m_path);
            fdist = -dist;
            auto ownLaneBackward = crdr.GetSrcLn();
            auto targetBackward = target.GetSrcLn();
            if (ownLaneBackward.IsValid() && targetBackward.IsValid()) {
                if (ownLaneBackward == targetBackward) {//if lanes are merging
                    float distBackward = -min(dist, float(target.GetLength()));
                    rt.Travel(distBackward, res.ids.rightForward, res.dist.rightForward, id);
                }
                else {
                    rt.Travel(fdist, res.ids.rightBackward, res.dist.rightBackward, id);
                }
            }

            fdist = dist;
            auto ownLaneForward = crdr.GetDstntnLn();
            auto targetForward = target.GetDstntnLn();
            CRoadTraveler rtf(pos, m_cCved, m_path);
            if (ownLaneForward.IsValid() && targetForward.IsValid()) {
                if (ownLaneForward == targetForward) {//if lanes are merging
                    float distForward = min(fdist, float(target.GetLength() - tdist));
                    rtf.Travel(distForward, res.ids.rightForward, res.dist.rightForward, id);
                }
                else {
                    rtf.Travel(fdist, res.ids.rightForward, res.dist.rightForward, id);
                }
            }
        }
    }
    /////////////////////////////////////
    //Left Lane
    ///////////////////////////////////// 
    leftPos = m_pos;
    if (leftPos.IsRoad()) {
        if (leftPos.ChangeLaneLeft()) {
            CRoadTraveler lt(leftPos, m_cCved, m_path);
            fdist = -dist;
            lt.Travel(fdist, res.ids.leftBackward, res.dist.leftBackward,id);
            CRoadTraveler ltf(leftPos, m_cCved, m_path);
            ltf.Travel(fdist, res.ids.leftForward,res.dist.leftForward,id);
        }
    }
    else {
        auto crdr = leftPos.GetCorridor();
        auto lane = crdr.GetSrcLn();
        TCrdrVec srcCrds, destCrds;
        if (lane.IsValid()) {
            CRoadPos rp(lane.GetRoad(), lane);
            if (rp.ChangeLaneLeft()) {
                auto intr = leftPos.GetIntrsctn();
                auto leftLane = rp.GetLane();
                intr.GetCrdrsStartingFrom(leftLane, srcCrds);
            }
            auto lanedest = crdr.GetDstntnLn();
            CRoadPos rp2(lanedest.GetRoad(), lanedest);
            if (rp2.IsValid() && rp2.ChangeLaneLeft()) {
                auto intr = leftPos.GetIntrsctn();
                auto leftLane = rp2.GetLane();
                intr.GetCrdrsLeadingTo(leftLane, destCrds);
            }
        }
        CCrdr target;
        for (auto itr = srcCrds.begin(); itr != srcCrds.end(); ++itr) {
            bool foundTarget = false;

            for (auto jtr = destCrds.begin(); jtr != destCrds.end(); ++jtr) {
                if (itr->GetId() == jtr->GetId()) {
                    target = *itr;
                    foundTarget = true;
                    break;
                }
                if (foundTarget) break;
            }
        }
        if (target.IsValid()) {
            float tdist = m_pos.GetDistance();
            if (tdist > target.GetLength()) {
                tdist = target.GetLength();
            }
            CRoadPos pos(target.GetIntrsctn(), target, tdist);
            CRoadTraveler lt(pos, m_cCved, m_path);
            fdist = -dist;
            auto ownLaneBackward = crdr.GetSrcLn();
            auto targetBackward = target.GetSrcLn();
            if (ownLaneBackward.IsValid() && targetBackward.IsValid()) {
                if (ownLaneBackward == targetBackward) {//if lanes are merging
                    float distBackward = -min(dist, float(target.GetLength()));
                    lt.Travel(distBackward, res.ids.rightForward, res.dist.rightForward, id);
                }
                else {
                    lt.Travel(fdist, res.ids.leftBackward, res.dist.leftBackward, id);
                }
            }
            CRoadTraveler ltf(pos, m_cCved, m_path);
            fdist = dist;

            auto ownLaneForward = crdr.GetDstntnLn();
            auto targetForward = target.GetDstntnLn();
            if (ownLaneForward.IsValid() && targetForward.IsValid()) {
                if (ownLaneForward == targetForward) {//if lanes are merging
                    float distForward = min(fdist, float(target.GetLength() - tdist));
                    ltf.Travel(distForward, res.ids.rightForward, res.dist.rightForward, id);
                }
                else {
                    ltf.Travel(fdist, res.ids.leftForward, res.dist.leftForward, id);
                }
            }
        }
    }
    return res;
}
