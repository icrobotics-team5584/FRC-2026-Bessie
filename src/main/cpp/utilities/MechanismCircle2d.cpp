#include "utilities/MechanismCircle2d.h"

void MechanismCircle2d::SetAngle(units::degree_t angle) {
    _indicatorLigament->SetAngle(angle);
    for (int i = 0; i < _spokes; i++) {
        _backgroundSpokeLigaments[i]->SetAngle(angle + (360_deg/_spokes) * i);
    }
}

void MechanismCircle2d::SetCircleColor(const frc::Color8Bit& color) {
    for (frc::MechanismLigament2d* spoke : _backgroundSpokeLigaments) {
        spoke->SetColor(color);
    }
}