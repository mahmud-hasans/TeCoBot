// Written by Mahmud Hasan Saikot

#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 imuBase;   // base  -> 0x4B
BNO080 imuTip;    // tip   -> 0x4A

#define BASE_ADDR 0x4B
#define TIP_ADDR  0x4A

struct Quaternion { float w,x,y,z; };

// ---------- Quaternion helpers ----------
Quaternion qMul(Quaternion a, Quaternion b){
  return { a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
           a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
           a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
           a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w };
}
Quaternion qInv(Quaternion q){ return { q.w, -q.x, -q.y, -q.z }; }

void rotateVec(const Quaternion& q, const float v[3], float out[3]){
  // r = q * v * q^-1
  Quaternion vq = {0, v[0], v[1], v[2]};
  Quaternion qi = qInv(q);
  Quaternion r  = qMul(qMul(q, vq), qi);
  out[0]=r.x; out[1]=r.y; out[2]=r.z;
}

float vnorm3(const float v[3]){ return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
void vscale3(float v[3], float s){ v[0]*=s; v[1]*=s; v[2]*=s; }
// ----------------------------------------

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  Serial.println("Initializing two BNO085s (SparkFun library) …");
  if (!imuBase.begin(BASE_ADDR, Wire)) { Serial.println("❌ Base IMU not found @0x4B"); while(1); }
  delay(100);
  if (!imuTip.begin(TIP_ADDR, Wire))  { Serial.println("❌ Tip  IMU not found @0x4A"); while(1); }

  // Use game rotation vector (gyro+accel, no mag) for stable relative orientation
  imuBase.enableGameRotationVector(100);
  imuTip.enableGameRotationVector(100);

  delay(400);
  Serial.println("✅ Both IMUs ready");
  Serial.println("alpha_deg\tpsi_tip_deg");
}

void loop() {
  if (!(imuBase.dataAvailable() && imuTip.dataAvailable())) { delay(2); return; }

  // ---- read quaternions ----
  Quaternion qB = { imuBase.getQuatReal(), imuBase.getQuatI(),
                    imuBase.getQuatJ(),    imuBase.getQuatK() };
  Quaternion qT = { imuTip.getQuatReal(),  imuTip.getQuatI(),
                    imuTip.getQuatJ(),     imuTip.getQuatK()  };

  // Relative rotation: base -> tip
  Quaternion qBT = qMul(qInv(qB), qT);   // expresses how to rotate BASE frame into TIP

  // ---- bending angle α (angle between z_B and z_T) ----
  float zB[3] = {0,0,1};
  float zT_in_B[3];               // tip z-axis expressed in BASE frame
  rotateVec(qBT, zB, zT_in_B);

  float c = zT_in_B[2];
  if (c > 1) c = 1; if (c < -1) c = -1;
  float alpha_deg = acosf(c) * 57.29578f;

  // ---- bending-plane direction (pure bend, no twist) ----
  // In the BASE frame, the bend direction (unit) is:
  // b_base = normalize( (zB x zT_B) x zB ), which lies in the BASE XY plane
  float n[3] = { -zT_in_B[1], zT_in_B[0], 0.0f };  // n = zB × zT_in_B
  float b_base[3] = { -n[1], n[0], 0.0f };         // b = n × zB = (-ny, nx, 0)

  float bn = vnorm3(b_base);
  static float psi_tip_prev = 0.0f;
  float psi_tip_deg;

  if (bn < 1e-6f) {
    // Near-straight configuration: bending plane undefined; hold last value
    psi_tip_deg = psi_tip_prev;
  } else {
    vscale3(b_base, 1.0f/bn);

    // Express this bend direction in the TIP plate frame
    // qTB = inverse of qBT maps BASE vectors into TIP coordinates
    Quaternion qTB = qInv(qBT);
    float b_tip[3];
    rotateVec(qTB, b_base, b_tip);     // now b_tip is in TIP XY plane ideally

    // Angle of b_tip in TIP XY
    psi_tip_deg = atan2f(b_tip[1], b_tip[0]) * 57.29578f;
    if (psi_tip_deg < 0) psi_tip_deg += 360.0f;

    psi_tip_prev = psi_tip_deg;
  }

  // ---- output ----
  Serial.print(alpha_deg, 2);
  Serial.print('\t');
  Serial.println(psi_tip_deg, 2);

  delay(20); // ~50 Hz
}
