/**
 * Contact mode annotator
 * @author Ted Li
 */


#include "../common/ContactMode.h"
#include "CircularBuffer.h"

#include <vector>

#define PREV_DIST_HIT  2
#define PREV_TIMESTEPS 50
#define ROLLING_Y_GROUND_THRESH 0.0125
#define ROLLING_Y_DISPLACEMENT_THRESH 0.0005
#define STATIC_THRESH 0.0001
#define HIT_ACC_MAG_THRESH 20.0

class ContactModeAnnotator {
private:
  CircularBuffer<float> _prev_x_pos, _prev_y_pos, _prev_z_pos;
  CircularBuffer<float> _prev_dist_ground, _prev_dist_wall1, _prev_dist_wall2;
  ContactMode _prev_contact_mode;

public:
  ContactModeAnnotator()
  {
    _prev_x_pos = CircularBuffer<float>(PREV_TIMESTEPS);
    _prev_y_pos = CircularBuffer<float>(PREV_TIMESTEPS);
    _prev_z_pos = CircularBuffer<float>(PREV_TIMESTEPS);
    _prev_dist_ground = CircularBuffer<float>(PREV_DIST_HIT);
    _prev_dist_wall1 = CircularBuffer<float>(PREV_DIST_HIT);
    _prev_dist_wall2 = CircularBuffer<float>(PREV_DIST_HIT);
    _prev_contact_mode = kUndefined;
  }

  float dist1(float x1, float x2)
  {
    return std::abs(x1 - x2);
  }

  float dist2(float x1, float y1, float x2, float y2)
  {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
  }

  float dist3(float x1, float y1, float z1, float x2, float y2, float z2)
  {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
  }

  ContactMode annotate(
    float x_pos, float y_pos, float z_pos,
    float y_vel,
    float x_acc, float y_acc, float z_acc,
    float dist_ground, float dist_wall1, float dist_wall2)
  {
    float acc_mag = dist3(0, 0, 0, x_acc, y_acc, z_acc);
    bool probably_rolling = true, probably_static = true;

    if (_prev_x_pos.size() == PREV_TIMESTEPS) {
      for (int i = 0; i < _prev_y_pos.size(); ++i) {
        if (dist1(y_pos, _prev_y_pos[i]) >= ROLLING_Y_DISPLACEMENT_THRESH ||
            dist1(0, dist_ground) >= ROLLING_Y_GROUND_THRESH) {
          probably_rolling = false;
          break;
        }
      }

      if (probably_rolling) {
        for (int i = 0; i < _prev_y_pos.size(); ++i) {
          if (dist3(x_pos, y_pos, z_pos,
            _prev_x_pos[i], _prev_y_pos[i], _prev_z_pos[i]) >= STATIC_THRESH) {
            probably_static = false;
            break;
          }
        }
      }

      if (_prev_contact_mode == kProbablyRolling) {
        probably_rolling = true;
      }

      if (_prev_contact_mode == kProbablyStatic) {
        probably_static = probably_rolling = true;
      }

      if (probably_rolling) {
        _prev_contact_mode = probably_static ? kProbablyStatic : kProbablyRolling;
        goto rtn;
      }
    }

    if (_prev_contact_mode == kHitContactGround ||
        _prev_contact_mode == kHitContactWall1 ||
        _prev_contact_mode == kHitContactWall2) {
      _prev_contact_mode = kBreakContact;
      goto rtn;
    }


    if (_prev_dist_ground.size() == 2) {
      if (_prev_dist_ground[1] < dist_ground &&
          _prev_dist_ground[1] < _prev_dist_ground[0] &&
          acc_mag >= HIT_ACC_MAG_THRESH) {
        _prev_contact_mode = kHitContactGround;
        goto rtn;
      }
      if (_prev_dist_wall1[1] < dist_wall1 &&
          _prev_dist_wall1[1] < _prev_dist_wall1[0] &&
          acc_mag >= HIT_ACC_MAG_THRESH) {
        _prev_contact_mode = kHitContactWall1;
        goto rtn;
      }
      if (_prev_dist_wall2[1] < dist_wall2 &&
          _prev_dist_wall2[1] < _prev_dist_wall2[0] &&
          acc_mag >= HIT_ACC_MAG_THRESH) {
        _prev_contact_mode = kHitContactWall2;
        goto rtn;
      }
    }

    if (y_vel > 0) {
      _prev_contact_mode = kMovingUpwards;
      goto rtn;
    }

    if (y_acc < 0) {
      _prev_contact_mode = kFreeFall;
      goto rtn;
    }

    _prev_contact_mode = kUndefined;

rtn:
    _prev_dist_ground.add(dist_ground);
    _prev_dist_wall1.add(dist_wall1);
    _prev_dist_wall2.add(dist_wall2);
    _prev_x_pos.add(x_pos);
    _prev_y_pos.add(y_pos);
    _prev_z_pos.add(z_pos);
    return _prev_contact_mode;
  }
};
