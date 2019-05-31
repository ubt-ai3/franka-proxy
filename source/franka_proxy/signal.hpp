/**
 *************************************************************************
 *
 * @file signal.hpp
 *
 * Signal class from viral_core by Tobias Werner.
 * Didn't want to use the whole library.
 *
 ************************************************************************/


#if !defined(INCLUDED__VIRAL_CORE_LITE__SIGNAL_HPP)
#define INCLUDED__VIRAL_CORE_LITE__SIGNAL_HPP


#include <mutex>
#include <condition_variable>


namespace viral_core_lite
{


/**
 *************************************************************************
 *
 * @class signal
 * @ingroup thread_module
 *
 * Allows client threads to block until a boolean variable
 * is set to some value or until some timeout has passed.
 *
 * Usage notes:
 * - A signal must not be destroyed while any threads are still waiting
 *		on the signal object. Otherwise, behaviour is undefined.
 * - Signals are threadsafe for an arbitrary number of threads at a time.
 *
 ************************************************************************/
class signal
{

	public:

		/** Create signal with initial value of @false. */
		signal();

		signal(const signal& other) = delete;


		~signal() noexcept;


		/** Atomic access to the internal value. */
		//@{
		void set(bool value) noexcept;
		bool get() const noexcept;
		//@}


		/**
		 * Set signal to @bref{value} and return previous value
		 * in one atomic call. This allows to determine whether
		 * a thread toggled the signal when there are multiple
		 * client threads.
		 */
		bool get_and_set(bool value) noexcept;


		/**
		 * Wait for signal @bref{value} either with default timeout
		 * or with user-specified @bref{timeout_seconds} hint.
		 *
		 * Usage notes:
		 * - Invalid @bref{timeout_seconds} silently clamp to valid range.
		 * - @attent Apart from signal state toggles, waiting also returns
		 *		at the default or user-set timeout and on external triggers
		 *		(e.g. OS scheduling). Therefore, clients must always recheck
		 *		the signal value after waiting returns.
		 * - @attent Waiting for a signal does not clear any other mutexes.
		 *		Thus clients still need to be careful to avoid deadlocks.
		 */
		//@{
		void wait_for(bool value) const noexcept;
		void wait_for(bool value, float timeout_seconds) const noexcept;
		//@}


	private:

		static const float default_timeout_seconds_;
		static const float max_timeout_seconds_;

		mutable std::mutex signal_mutex_;
		bool signal_value_;

		mutable std::mutex condition_mutex_;
		mutable std::condition_variable condition_toggle_;
};




} /* namespace viral_core_lite */


#endif /* !defined(INCLUDED__VIRAL_CORE_LITE__SIGNAL_HPP) */