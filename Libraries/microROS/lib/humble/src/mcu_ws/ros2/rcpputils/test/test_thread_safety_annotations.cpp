// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"

#include "rcpputils/thread_safety_annotations.hpp"

/*
Macros tested

Regular Capability
[X] CAPABILITY
[X] SCOPED_CAPABILITY
[X] GUARDED_BY
[X] REQUIRES
[X] ACQUIRE
[X] RELEASE
[X] ASSERT_CAPABILITY
[X] EXCLUDES

Ptr
[X] PT_GUARDED_BY

Shared Capability
[X] REQUIRES_SHARED
[X] ACQUIRE_SHARED
[X] RELEASE_SHARED
[X] ASSERT_SHARED_CAPABILITY

Other
[X] RETURN_CAPABILITY
[X] NO_THREAD_SAFETY_ANALYSIS

see note in test try_acquire
[ ] TRY_ACQUIRE
[ ] TRY_ACQUIRE_SHARED

see note in test acquire_ordering
[ ] ACQUIRED_BEFORE
[ ] ACQUIRED_AFTER
*/

struct StdGuarded
{
  std::mutex mu;
  int data RCPPUTILS_TSA_GUARDED_BY(mu);

  int incr() RCPPUTILS_TSA_REQUIRES(mu) {
    return ++data;
  }

  int incr_Unsafe() RCPPUTILS_TSA_NO_THREAD_SAFETY_ANALYSIS
  {
    return data++;
  }
};

struct RCPPUTILS_TSA_CAPABILITY ("mutex") FakeMutex
{
  bool locked = false;
  int readerLocks = 0;

  void lock() RCPPUTILS_TSA_ACQUIRE()
  {
    locked = true;
  }

  void unlock() RCPPUTILS_TSA_RELEASE()
  {
    locked = false;
  }

  void readerLock() RCPPUTILS_TSA_ACQUIRE_SHARED()
  {
    // NOTE: This is not even trying to pretend to be correct
    readerLocks++;
  }

  void readerUnlock() RCPPUTILS_TSA_RELEASE_SHARED()
  {
    // NOTE: This is not even trying to pretend to be correct
    readerLocks--;
  }

  void assertHeld() RCPPUTILS_TSA_ASSERT_CAPABILITY()
  {
    assert(locked);
  }

  void assertReaderHeld() RCPPUTILS_TSA_ASSERT_SHARED_CAPABILITY()
  {
    assert(readerLocks > 0);
  }
};

struct RCPPUTILS_TSA_SCOPED_CAPABILITY FakeLockGuard
{
  FakeMutex * mutex_;

  explicit FakeLockGuard(FakeMutex * mutex)
  RCPPUTILS_TSA_ACQUIRE(mutex)
  : mutex_(mutex)
  {
    if (!mutex_) {return;}
    mutex_->lock();
  }

  ~FakeLockGuard()
  RCPPUTILS_TSA_RELEASE()
  {
    if (!mutex_) {return;}
    mutex_->unlock();
  }
};

struct FakeGuarded
{
  FakeMutex mu;
  int data RCPPUTILS_TSA_GUARDED_BY(mu) = 0;
  int * pData RCPPUTILS_TSA_PT_GUARDED_BY(mu);

  FakeGuarded()
  {
    pData = new int;
    *pData = 0;
  }

  ~FakeGuarded()
  {
    delete pData;
  }

  int incr_HaveGuard() RCPPUTILS_TSA_REQUIRES(mu)
  {
    return data++;
  }

  int get_NoHaveGuard() RCPPUTILS_TSA_EXCLUDES(mu)
  {
    std::lock_guard<FakeMutex> lock(mu);
    return data;
  }

  int get_Shared() RCPPUTILS_TSA_REQUIRES_SHARED(mu)
  {
    return data;
  }
};

struct PrivateFakeGuarded
{
  FakeMutex * getMutex()
  RCPPUTILS_TSA_RETURN_CAPABILITY(mu)
  {
    return &mu;
  }

  FakeMutex * abuseGetMutex()
  RCPPUTILS_TSA_RETURN_CAPABILITY(mu)
  {
    return nullptr;
  }

  int data RCPPUTILS_TSA_GUARDED_BY(mu) = 0;

  void doSomething() RCPPUTILS_TSA_REQUIRES(getMutex())
  {
    data++;
  }

private:
  FakeMutex mu;
};

TEST(test_tsa, libcxx_types) {
  // Test all the annotations provided by libcxx currently
  StdGuarded guarded;

  // If you remove either following call you should get a build warning
  guarded.mu.lock();
  guarded.data++;
  guarded.mu.unlock();

  {
    // If you remove the lock_guard, you should get a build warning
    std::lock_guard<std::mutex> lock(guarded.mu);
    guarded.data++;
    guarded.incr();
    // ASSERT_EQ(guarded.data, 3);
  }

  // tests NO_THREAD_SAFETY_ANALYSIS
  guarded.incr_Unsafe();
}

TEST(test_tsa, capability) {
  // Test our macros

  // tests CAPABILITY
  FakeGuarded guarded;

  // tests ACQUIRE
  guarded.mu.lock();

  // tests GUARDED_BY
  guarded.data++;

  // tests REQUIRES
  guarded.incr_HaveGuard();

  // tests ASSERT_CAPABILITY
  guarded.mu.assertHeld();

  // tests RELEASE
  guarded.mu.unlock();

  // tests EXCLUDES
  // NOTE: ASSERT calls while lock is held will return
  // "mutex is not held on every path through here"
  // becaue the expansion has an early return
  ASSERT_EQ(guarded.get_NoHaveGuard(), 2);

  {
    // tests SCOPED_CAPABILITY
    FakeLockGuard lock(&guarded.mu);
    guarded.incr_HaveGuard();
  }
}

TEST(test_tsa, ptr_guard) {
  FakeGuarded guarded;

  // tests PT_GUARDED_BY
  guarded.mu.lock();
  *guarded.pData = 2;
  guarded.mu.unlock();

  int * old = guarded.pData;
  guarded.pData = new int;  // pData itself is not protected by the mutex
  delete old;
}

TEST(test_tsa, shared_capability) {
  FakeGuarded guarded;

  // tests ACQUIRE_SHARED
  guarded.mu.readerLock();
  // tests ASSERT_SHARED_CAPABILITY
  guarded.mu.assertReaderHeld();
  // tests REQUIRES_SHARED
  guarded.get_Shared();
  // tests RELEASE_SHARED
  guarded.mu.readerUnlock();
}

void doSomethingTwice(PrivateFakeGuarded & guarded) RCPPUTILS_TSA_REQUIRES(guarded.getMutex())
{
  guarded.doSomething();
  guarded.doSomething();
}

TEST(test_tsa, return_capability) {
  PrivateFakeGuarded guarded;
  // this pattern does not work - you can't give the returned capability a name
  {
    // auto mu = guarded.getMutex();
    // mu->lock();
    // guarded.clear();
    // mu->unlock();
  }

  // Normal pattern
  {
    // tests SCOPED_CAPABILITY and RETURN_CAPABILITY
    FakeLockGuard lock(guarded.getMutex());
    guarded.doSomething();
  }

  // Recommended abuse pattern recommended in the docs
  // https://clang.llvm.org/docs/ThreadSafetyAnalysis.html#private-mutexes
  {
    FakeLockGuard lock(guarded.abuseGetMutex());
    doSomethingTwice(guarded);
  }
}

TEST(test_tsa, try_acquire) {
  /*
  TRY doesn't seem to be working and therefore untested. Timeline pieced together (possibly incorrect)
  - 03-2018 - LLVM 6.0.0 released
  - 04-2018 - TryLock bug reported
    - https://bugs.llvm.org/show_bug.cgi?id=32954
  - 04-2018 - TryLock bug fixed
    - svn info -r 329930 http://llvm.org/svn/llvm-project/llvm/
  - 05-2018 - Ubuntu Bionic clang6 package released
    - http://archive.ubuntu.com/ubuntu/ubuntu/pool/universe/l/llvm-defaults/
  - 07-2018 - LLVM 6.0.1 released
  */
}

TEST(test_tsa, acquire_ordering)
{
  // https://clang.llvm.org/docs/ThreadSafetyAnalysis.html#acquired-before-and-acquired-after-are-currently-unimplemented
}
