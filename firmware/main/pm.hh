#pragma once

#include <functional>

template <typename T> struct Pm {
  struct {
    T pm1Mcg;
    T pm2Mcg;
    T pm10Mcg;
  } std;
  struct {
    T pm1Mcg;
    T pm2Mcg;
    T pm10Mcg;
  } atm;
  struct {
    T pm03Count;
    T pm05Count;
    T pm1Count;
    T pm2Count;
    T pm5Count;
    T pm10Count;
  } cnt;

  template <typename R> Pm &operator=(const Pm<R> &rhs) {
    return map<R>(
        rhs, [](const R in, const T out) -> T { return static_cast<T>(in); });
  }

  template <typename R> Pm &operator+=(const Pm<R> &rhs) {
    return map<R>(rhs, [](const R in, const T out) -> T {
      return static_cast<T>(in) + out;
    });
  }

  template <typename R> Pm<T> &operator/=(const R divisor) {
    return map<R>([divisor](const T val) { return val / divisor; });
  }

private:
  template <typename R> using BinaryMapper = std::function<T(R in, T out)>;
  template <typename R> using UnaryMapper = std::function<T(T val)>;

  template <typename R> Pm<T> &map(const UnaryMapper<R> &mapper) {
    T *dst = reinterpret_cast<T *>(this);
    T *const dstEnd = dst + sizeof(Pm<T>) / sizeof(T);

    std::transform(dst, dstEnd, dst, mapper);

    return *this;
  }

  template <typename R>
  Pm<T> &map(const Pm<R> &rhs, const BinaryMapper<R> &mapper) {
    T *dst = reinterpret_cast<T *>(this);
    const R *src = reinterpret_cast<const R *>(&rhs);
    const R *const srcEnd = src + sizeof(Pm<R>) / sizeof(R);

    for (; src != srcEnd; ++dst, ++src) {
      *dst = mapper(*src, *dst);
    }

    return *this;
  }
} __attribute__((packed));
