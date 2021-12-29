/*
 * @Descripttion: copy from ethz-asl/maplab & ethz-asl/aslam
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-27 00:47:52
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-09-14 02:02:46
 */
#pragma once

#include <memory>
// #include <type_traits>
//
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace internal {
template <typename Type>
struct aligned_delete {
  constexpr aligned_delete() noexcept = default;

  template <typename TypeUp,
            typename = typename std::enable_if<
                std::is_convertible<TypeUp*, Type*>::value>::type>
  aligned_delete(const aligned_delete<TypeUp>&) noexcept {}

  void operator()(Type* ptr) const {
    static_assert(sizeof(Type) > 0, "Can't delete pointer to incomplete type!");
    typedef typename std::remove_const<Type>::type TypeNonConst;
    Eigen::aligned_allocator<TypeNonConst> allocator;
    allocator.destroy(ptr);
    allocator.deallocate(ptr, 1u /*num*/);
  }
};
}  // namespace internal

template <typename Type>
using AlignedUniquePtr = std::unique_ptr<
    Type, internal::aligned_delete<typename std::remove_const<Type>::type>>;

template <typename Type, typename... Arguments>
inline AlignedUniquePtr<Type> aligned_unique(Arguments&&... arguments) {
  typedef typename std::remove_const<Type>::type TypeNonConst;
  Eigen::aligned_allocator<TypeNonConst> allocator;
  TypeNonConst* obj = allocator.allocate(1u);
  allocator.construct(obj, std::forward<Arguments>(arguments)...);
  return std::move(AlignedUniquePtr<Type>(obj));
}

#define POINTER_TYPEDEFS(TypeName)                    \
  typedef std::shared_ptr<TypeName> Ptr;              \
  typedef std::shared_ptr<const TypeName> ConstPtr;   \
  typedef AlignedUniquePtr<TypeName> UniquePtr;       \
  typedef std::weak_ptr<TypeName> WeakPtr;            \
  typedef std::weak_ptr<const TypeName> WeakConstPtr; \
  void definePointerTypedefs##__FILE__##__LINE__(void)

// Disallow implicit argument type conversion at function call.
#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;
