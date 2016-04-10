/**
 * Contact modes
 *
 * @author Ted Li
 */

typedef enum {
  kUndefined = 0,
  kFreeFall = 1,
  kHitContactGround = 2,
  kHitContactWall1 = 3,
  kHitContactWall2 = 4,
  kBreakContact = 5,
  kMovingUpwards = 6,
  kProbablyStatic = 7,
} ContactMode;
