#include "wiringPi.h"
#include <math.h>

template<typename T, size_t T_size>
struct Array {

	T _data[T_size];

	constexpr inline T& operator[](size_t index) noexcept { return _data[index]; }
};

typedef double Float;

#define FLOAT(x) static_cast<Float>(x)

static constexpr const Float pi = static_cast<Float>(3.14159265358979323846);

struct Vec2 {
	Float x, y;
	constexpr inline Vec2(Float x = FLOAT(0), Float y = FLOAT(0)) noexcept : x(x), y(y) {}
};

typedef Vec2 Euler2;

struct Vec3 {

	Float x, y, z;

	constexpr inline Vec3(Float x = FLOAT(0), Float y = FLOAT(0), Float z = FLOAT(0)) noexcept : x(x), y(y), z(z) {}

	constexpr inline Vec3 Normalized() const { return Vec3(); }

	static constexpr inline Vec3 Up() noexcept { return Vec3(FLOAT(0), FLOAT(1), FLOAT(0)); }
	static constexpr inline Vec3 Cross(const Vec3& a, const Vec3& b) noexcept { return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
};

struct Quaternion {

	Float x, y, z, w;

	constexpr inline Quaternion(Float x = FLOAT(0), Float y = FLOAT(0), Float z = FLOAT(0), Float w = FLOAT(1)) noexcept : x(x), y(y), z(z), w(w) {}

	constexpr inline Euler2 AsEuler2() noexcept {

		Euler2 res{};

		Float sinRoll = FLOAT(2) * (FLOAT(2) * x + y * z);
		Float cosRoll = FLOAT(1) - FLOAT(2) * (x * x + y * y);
		res.x = atan2(sinRoll, cosRoll);

		Float sinPitch = FLOAT(2) * (w * y - z * x);
		res.y = abs(sinPitch) >= FLOAT(1) ? copysign(pi, sinPitch) : asin(sinPitch);

		return res;
	}
};

struct Mat3 {

	Array<Array<Float, 3>, 3> m{};

	constexpr inline Quaternion ToQuaternion() noexcept {

		Float num0 = m[0][0] - m[1][1] - m[2][2];
		Float num1 = m[1][1] - m[0][0] - m[2][2];
		Float num2 = m[2][2] - m[0][0] - m[1][1];
		Float num3 = m[0][0] + m[1][1] + m[2][2];

		Float biggestNum = num0;
		size_t biggestIndex = 0;
		if (num1 > biggestNum) {
			biggestNum = num1;
		}
		if (num2 > biggestNum) {
			biggestNum = num2;
		}
		if (num3 > biggestNum) {
			biggestNum = num3;
		}

		Float biggestVal = sqrt(biggestNum - FLOAT(1)) * FLOAT(0.5);
		Float mult = FLOAT(0.25) / biggestVal;

		switch(biggestIndex) {
			case 0:
				return Quaternion(biggestVal, (m[0][1] + m[1][0]) * mult, (m[2][0] + m[0][2]) * mult, (m[1][2] - m[2][1]) * mult);
			case 1:
				return Quaternion((m[0][1] + m[1][0]) * mult, biggestVal, (m[1][2] + m[2][1]) * mult, (m[2][0] - m[0][2]) * mult);
			case 2:
				return Quaternion((m[2][0] + m[0][2]) * mult, (m[1][2] + m[2][1]) * mult, biggestVal, (m[0][1] - m[1][0]) * mult);
			case 3:
				return Quaternion((m[1][2] - m[2][1]) * mult, (m[2][0] - m[0][2]) * mult, (m[0][1] - m[1][0]) * mult, biggestVal);
			default:
				return Quaternion();
		}
	}

	constexpr inline Mat3 LookAt(const Vec3& front) noexcept {
		Vec3 up = Vec3::Up();
		Vec3 right = Vec3::Cross(up, front).Normalized();
		up = Vec3::Cross(front, right).Normalized();
		Mat3 res{};
		res[0][0] = right.x;
		res[1][0] = right.y;
		res[2][0] = right.z;
		res[0][1] = up.x;
		res[1][1] = up.y;
		res[2][1] = up.z;
		res[0][2] = front.x;
		res[1][2] = front.y;
		res[2][2] = front.z;
		return res;
	}

	constexpr inline Array<Float, 3>& operator[](size_t index) noexcept {
		return m[index];
	}
};

static Euler2 euler2Rotation(FLOAT(0), FLOAT(0));
static Quaternion quaternionRot(FLOAT(0), FLOAT(0), FLOAT(0), FLOAT(1));

int main() {
	return 0;
}
