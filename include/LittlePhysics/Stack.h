#pragma once
#include "Core.h"
#include "DataTypes.h"

namespace LP {

	template <typename T>
	class LP_API Stack
	{
	public:
		Stack(uint32 capacity);
		~Stack();
		void Push(const T& data);
		void Pop();
		T& Top();
		uint32 Size() const;
		bool Empty() const;
	private:
		T* m_Data = nullptr;
		uint32 m_Size = 0;
		uint32 m_Capacity = 0;
	};
	template<typename T>
	inline Stack<T>::Stack(uint32 capacity)
	{
		if (capacity <= 8)
			capacity = 8;
		m_Capacity = capacity;
		m_Size = 0;
		m_Data = (T*)::operator new(capacity * sizeof(T));
	}
	template<typename T>
	inline Stack<T>::~Stack()
	{
		delete[] m_Data;
	}
	template<typename T>
	inline void Stack<T>::Push(const T& data)
	{
		if (m_Size == m_Capacity)
		{
			uint32 newCapacity = m_Capacity * 2;
			T* newBlock = (T*)::operator new(newCapacity * sizeof(T));
			for (uint32 i = 0; i < m_Capacity; i++)
			{
				newBlock[i] = std::move(m_Data[i]);
			}
			for (uint32 i = 0; i < m_Capacity; i++)
			{
				m_Data[i].~T();
			}
			::operator delete(m_Data, m_Capacity * sizeof(T));
			m_Data = newBlock;
			m_Capacity = newCapacity;
		}
		m_Data[m_Size] = std::move(data);
		m_Size++;
	}
	template<typename T>
	inline void Stack<T>::Pop()
	{
		m_Data[m_Size].~T();
		m_Size--;
	}
	template<typename T>
	inline T& Stack<T>::Top()
	{
		return m_Data[m_Size - 1];
	}
	template<typename T>
	inline uint32 Stack<T>::Size() const
	{
		return m_Size;
	}
	template<typename T>
	inline bool Stack<T>::Empty() const
	{
		return m_Size == 0;
	}
}