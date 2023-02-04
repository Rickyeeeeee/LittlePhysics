#pragma once
#include "Core.h"
#include "DataTypes.h"
#include <cmath>

namespace LP {

	struct LP_API Vec2
	{
		float x;
		float y;

		Vec2() = default;
		Vec2(float x, float y) : x(x), y(y) {}
		Vec2(float value) : x(value), y(value) {}
		//Vec2(const Vec2& other) : x(other.x), y(other.y) {]}
		//Vec2(Vec2&& other):

		bool operator == (const Vec2& other) const
		{
			return x == other.x && y == other.y;
		}

		Vec2& operator *= (float m)
		{
			x *= m;
			y *= m;
			return *this;
		}
		Vec2& operator *= (const Vec2& other)
		{
			x *= other.x;
			y *= other.y;
			return *this;
		}

		Vec2& operator /= (const Vec2& other)
		{
			x /= other.x;
			y /= other.y;
			return *this;
		}

		Vec2& operator += (const Vec2& other)
		{
			x += other.x;
			y += other.y;
			return *this;
		}

		Vec2& operator -= (const Vec2& other)
		{
			x -= other.x;
			y -= other.y;
			return *this;
		}

		Vec2 operator * (const Vec2& other) const
		{
			return Vec2{ x * other.x, y * other.y };
		}

		Vec2 operator * (float m) const
		{
			return Vec2{ x * m, y * m };
		}

		Vec2 operator / (const Vec2& other) const
		{
			return Vec2{ x / other.x, y / other.y };
		}

		Vec2 operator / (float m) const
		{
			return Vec2{ x / m, y / m };
		}

		Vec2 operator + (const Vec2& other) const
		{
			return Vec2{ x + other.x, y + other.y };
		}

		Vec2 operator - (const Vec2& other) const
		{
			return Vec2{ x - other.x, y - other.y };
		}

		float& operator[] (const uint32 i) 
		{
			return (&x)[i];
		}

		float operator[] (const uint32 i) const
		{
			return (&x)[i];
		}

		Vec2 operator- () const
		{
			return Vec2{ -x, -y };
		}

		inline Vec2 Normalize() const
		{
			return (*this) / Length();
		}

		inline float Length() const
		{
			return sqrtf(x * x + y * y);
		}

		inline float Length2() const
		{
			return x * x + y * y;
		}

		inline float Dot(Vec2 v) const
		{
			return x * v.x + y * v.y;
		}

		inline float Cross(const Vec2& v) const
		{
			return this->x * v.y - this->y * v.x;
		}

		inline Vec2 Cross(float v) const
		{
			return { -v * this->y, v * this->x };
		}
	};

	// row major
	struct LP_API Mat2x2
	{
		Mat2x2() = default;
		Mat2x2(Vec2 ex, Vec2 ey) : Ex(ex), Ey(ey) {}
		Mat2x2(float a, float b, float c, float d)
		{
			Ex.x = a, Ex.y = b;
			Ey.x = c, Ey.y = d;
		}
		Vec2 operator* (const Vec2& v) const
		{
			Vec2 q;
			q.x = Ex.x * v.x + Ex.y * v.y;
			q.y = Ey.x * v.x + Ey.y * v.y;
			return q;
		}
		Vec2 Ex, Ey;
	};

	struct LP_API Rot
	{
		float Cos, Sin;
		Rot() : Cos(1.0f), Sin(0.0f) {}
		Rot(float angle) : Cos(cosf(angle)), Sin(sinf(angle)) {}
		Rot(float cos, float sin) : Cos(cos), Sin(sin) {}
		void Set(float angle)
		{
			Sin = sinf(angle);
			Cos = cosf(angle);
		}
		Vec2 Axis() const
		{
			return Vec2{ Cos, Sin };
		}
		Vec2 AxisPer() const
		{
			return Vec2{ -Sin, Cos };
		}
		Mat2x2 GetMatrix() const
		{
			return Mat2x2{ Cos, -Sin, Sin, Cos };
		}
		float GetAngle() const
		{
			return atan2f(Sin, Cos);
		}
		Rot operator- () const
		{
			return Rot{ Cos, -Sin };
		}
	};

	struct LP_API Transform
	{
		Vec2 P = { 0.0f, 0.0f };
		Rot R;
	};

	#define RotMatrix(rot) Mat2x2{ rot.Cos, -rot.Sin, rot.Sin, rot.Cos }
}