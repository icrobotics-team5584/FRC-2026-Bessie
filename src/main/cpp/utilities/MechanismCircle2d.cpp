#include "utilities/MechanismCircle2d.h"

void MechanismCircle2d::SetAngle(units::degree_t angle) {
    int spokes = _backgroundSpokeLigaments.size();
    _indicatorLigament->SetAngle(angle);
    for (unsigned int i = 0; i < spokes; i++) {
        _backgroundSpokeLigaments[i]->SetAngle(angle + (360_deg/spokes) * i);
    }
}

void MechanismCircle2d::SetIndicatorColor(const frc::Color8Bit& color) {
    _indicatorLigament->SetColor(color);
}

void MechanismCircle2d::SetCircleColor(const frc::Color8Bit& color) {
    for (frc::MechanismLigament2d* spoke : _backgroundSpokeLigaments) {
        spoke->SetColor(color);
    }
}