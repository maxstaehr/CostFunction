#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace Microsoft::VisualStudio::TestTools::UnitTesting;


namespace TestProjekt
{
	[TestClass]
	public ref class UnitTest1
	{
	private:
		TestContext^ testContextInstance;

	public: 
		/// <summary>
		///Ruft den Textkontext mit Informationen �ber
		///den aktuellen Testlauf sowie Funktionalit�t f�r diesen auf oder legt diese fest.
		///</summary>
		property Microsoft::VisualStudio::TestTools::UnitTesting::TestContext^ TestContext
		{
			Microsoft::VisualStudio::TestTools::UnitTesting::TestContext^ get()
			{
				return testContextInstance;
			}
			System::Void set(Microsoft::VisualStudio::TestTools::UnitTesting::TestContext^ value)
			{
				testContextInstance = value;
			}
		};

		#pragma region Additional test attributes
		//
		//Sie k�nnen beim Schreiben der Tests folgende zus�tzliche Attribute verwenden:
		//
		//Mit ClassInitialize f�hren Sie Code aus, bevor Sie den ersten Test in der Klasse ausf�hren.
		//[ClassInitialize()]
		//static void MyClassInitialize(TestContext^ testContext) {};
		//
		//Mit ClassCleanup f�hren Sie Code aus, nachdem alle Tests in einer Klasse ausgef�hrt wurden.
		//[ClassCleanup()]
		//static void MyClassCleanup() {};
		//
		//Mit TestInitialize k�nnen Sie vor jedem einzelnen Test Code ausf�hren.
		//[TestInitialize()]
		//void MyTestInitialize() {};
		//
		//Mit TestCleanup k�nnen Sie nach jedem Test Code ausf�hren.
		//[TestCleanup()]
		//void MyTestCleanup() {};
		//
		#pragma endregion 

		[TestMethod]
		void Instance()
		{
			//
			// TODO: Testlogik hier einf�gen
			//
			HomogeneTransformation h;
			Assert::IsNotNull(h);

		};

		[TestMethod]
		void SetGet()
		{
			//
			// TODO: Testlogik hier einf�gen
			//
			HomogeneTransformation h;
			Assert::IsNull(h); 
			//float t[] = {1.f 1.f 1.f 1.f
			//	1.f 1.f 1.f 1.f
			//	1.f 1.f 1.f 1.f
			//	1.f 1.f 1.f 1.f };
		};


	};
}
