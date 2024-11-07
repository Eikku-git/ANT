#include "wiringPi.h"
#include <time.h>
#include <math.h>

template<typename T, size_t T_size>
struct Array {

	T _data[T_size];

	constexpr inline T& operator[](size_t index) noexcept { return _data[index]; }
};

typedef double Float;

#define FLOAT(x) static_cast<Float>(x)

static inline Float Clamp(Float val, Float min, Float max) {
	val = val < min ? min : val;
	return val > max ? max : val;
}

static constexpr const Float pi = FLOAT(3.14159265358979323846);

struct Vec2 {
	Float x, y;
	constexpr inline Vec2(Float x = FLOAT(0), Float y = FLOAT(0)) noexcept : x(x), y(y) {}
	constexpr inline Vec2 operator-(const Vec2& other) const {
		return Vec2(x - other.x, y - other.y);
	}
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

	static constexpr inline Float AngleBetween(const Quaternion& a, const Quaternion& b) noexcept {
		return acos(fmin(fabs(Quaternion::Dot(a, b)), FLOAT(1))) * FLOAT(2);
	}

	static constexpr inline Quaternion Slerp(const Quaternion& from, const Quaternion& to, Float t) noexcept {
		return Quaternion(from * (1 - t) + to * t);
	}

	static constexpr inline Quaternion RotateTowards(const Quaternion& from, const Quaternion& to, Float maxRadians) noexcept {
		float angle = AngleBetween(from, to);
		if (abs(angle) < 0.000001f) {
			return to;
		}
		return Slerp(from, to, Clamp(maxRadians / angle, FLOAT(0), FLOAT(1)));
	}

	static constexpr inline Float Dot(const Quaternion& a, const Quaternion& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

	constexpr inline Euler2 AsEuler2() noexcept {

		Euler2 res{};

		Float sinRoll = FLOAT(2) * (FLOAT(2) * x + y * z);
		Float cosRoll = FLOAT(1) - FLOAT(2) * (x * x + y * y);
		res.x = atan2(sinRoll, cosRoll);

		Float sinPitch = FLOAT(2) * (w * y - z * x);
		res.y = abs(sinPitch) >= FLOAT(1) ? copysign(pi, sinPitch) : asin(sinPitch);

		return res;
	}

	constexpr inline Quaternion operator+(const Quaternion& other) const noexcept { return Quaternion(x + other.x, y + other.y, z + other.z, w + other.w); }
	constexpr inline Quaternion operator*(float scalar) const noexcept { return Quaternion(x * scalar, y * scalar, z * scalar, w * scalar); }
};

struct Mat3 {

	Array<Array<Float, 3>, 3> m{};

	constexpr inline Quaternion AsQuaternion() noexcept {

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

	static constexpr inline Mat3 LookAt(const Vec3& front) noexcept {
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

	constexpr inline Vec3 operator*(const Vec3& vector) noexcept {
		return Vec3();
	}
};

static Euler2 euler2Rotation(FLOAT(0), FLOAT(0));
static Quaternion quaternionRotation(FLOAT(0), FLOAT(0), FLOAT(0), FLOAT(1));
static constexpr const Float max_rotation_speed_per_second = FLOAT(5);
static constexpr const Vec2 pitch_motor_constraints(FLOAT(-pi / 6), FLOAT(pi / 4));
static Float deltaTime = FLOAT(0);

struct Target {
	Vec3 direction;
	Float depth;
};

void GetTargets(size_t* outCount, Target** ppOutTargets) {
}

bool RotateMotors(const Vec2& amount) {
	return false;
}

int main() {
	while (true) {
		size_t targetCount;
		Target* targets;
		GetTargets(&targetCount, &targets);
		if (targetCount) {
			size_t closestIndex = 0;
			Target& closestTarget = targets[0];
			for (size_t i = 1; i < targetCount; i++) {
				if (targets[i].depth < closestTarget.depth) {
					closestIndex = i;
					closestTarget = targets[i];
				}
			}
			Quaternion lookAtRotation = Mat3::LookAt(closestTarget.direction.Normalized()).AsQuaternion();
			Quaternion rotationThisFrame = Quaternion::RotateTowards(quaternionRotation, lookAtRotation, max_rotation_speed_per_second * deltaTime);
			Euler2 euler2ThisFrame = rotationThisFrame.AsEuler2();
			if (euler2ThisFrame.x > pitch_motor_constraints.x && euler2ThisFrame.x < pitch_motor_constraints.y) {
				if (RotateMotors(euler2Rotation - euler2ThisFrame)) {
					quaternionRotation = rotationThisFrame;
					euler2Rotation = euler2ThisFrame;
				}
				else {
					// print or something
				}
			}
		}
		else {
			// patroll
		}
	}
	return 0;
}
