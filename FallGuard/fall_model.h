// Fall classifier. ml/train.py OVERWRITES this file with a trained model.
// This stub means "no model trained yet": the detector uses the rules in config.h.
//
// Feature order (must match FEATURES in ml/train.py):
//   f[0] peak |a| (g)          f[1] min |a| from ~1 s before impact to end of impact window (g)
//   f[2] peak |w| (dps)        f[3] still fraction after impact (0..1)
//   f[4] orientation change (deg)
#pragma once
#define FALL_MODEL_TRAINED 0
inline bool fallModelPredict(const float* f) { (void)f; return false; }
