#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

namespace TestProjekt
{
	[TestClass]
	public ref class UnitTest1
	{
	public: 
		[TestMethod]
		void TestMethod1()
		{
			Assert::IsNotNull(NULL);
		}
	};
}
