#include "Enumerations.h"

struct FootNote {
private:
    FootStance stance;
    Point anchor_point_oCfF; // FLOOR frameFrame::BODY origin. The position at which a foot touches the ground

public:
    FootNote() = default;

    FootNote(Point new_anchor_point_oCfF) {
        stance = FootStance::PLANTED;
        anchor_point_oCfF = new_anchor_point_oCfF;
    }

    void setAnchorPoint(Point anchor) {
        anchor_point_oCfF = anchor;
    }

    void switchStance(FootStance new_stance) {
        if (new_stance == stance) {
            return;
        }

        stance = new_stance;
    }

    Point getAnchorPoint_oCfF() {
        if (stance == FootStance::LIFTED) {
            return POINT_NULL;
        }
        return anchor_point_oCfF;
    }

    FootStance getStance() {
        return stance;
    }

    bool isAnchored() {
        return !(stance == FootStance::LIFTED);
    }
};