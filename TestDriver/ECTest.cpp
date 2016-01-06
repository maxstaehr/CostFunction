#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;


#include "../CFObject/ECTestAdapter.h"


namespace TestDriver
{
	[TestClass]
	public ref class ECTest
	{
	public: 

		[TestMethod]
		void TestConstuctor()
		{
			ECTestAdapter adapter;			
			Assert::IsTrue(adapter.test1());
		}
	};
}
