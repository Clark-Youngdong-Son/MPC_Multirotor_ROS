#include "data_logger.h"

using namespace TNT;
using namespace std;

typedef string String;

namespace hss
{

DataLogger::DataLogger()
{
	mDir = ".";
	mFilename = "datalog.txt";
	mLogStream = NULL;

	mPaused = false;
	mRunning = false;
	mDone = true;
	
	mLogStream = new ofstream;
    mLogStream->open(mDir+"/"+mFilename);
}

DataLogger::DataLogger(const string &dir, const string &name)
{
	mDir = dir;
	mFilename = name + ".txt";
	mLogStream = NULL;

	mPaused = false;
	mRunning = false;
	mDone = true;
	
	mLogStream = new ofstream;
    mLogStream->open(mDir+"/"+mFilename);
}

DataLogger::~DataLogger()
{
	shutdown();
    delete mLogStream;
}

void DataLogger::run()
{
	mDone = false;
	mRunning = true;

	string str = "";
	for(int i=0; i<80; i++)
		str = str+ to_string(i) +"\t";
	str = str + "\n";
	*mLogStream << str;

	// for easy indexing while post processing
	str = "0\t-500\n";
	*mLogStream << str;

	//while(mLogQueue.size() > 0)
	//	mLogQueue.pop_front();

	string line;
	shared_ptr<LogEntry> entry;
	
	while(mRunning)
	{
		mMutex_logQueue.lock(); 
		int size = mLogQueue.size();
		mMutex_logQueue.unlock();

		while(size > 0 && !mPaused &&  mRunning)
		{
			mMutex_logQueue.lock();
			entry  = mLogQueue.front();
			mLogQueue.pop_front();
			mMutex_logQueue.unlock();

			line = "";
			line = line + to_string(entry->timestamp) + "\t";
			line = line + to_string(entry->id) + "\t";
			line = line + entry->str;
			line = line + "\n";

			if(mLogStream != NULL)
				*mLogStream << line;
			
			mMutex_logQueue.lock();
			size = mLogQueue.size();
			mMutex_logQueue.unlock();
		}

		this_thread::sleep_for( chrono::milliseconds(100) );
	}

	// Clear the remainder of the queue
	mMutex_logQueue.lock(); 
	int size = mLogQueue.size(); 
	mMutex_logQueue.unlock();
	
	while(size > 0)
	{
		mMutex_logQueue.lock();
		entry  = mLogQueue.front();
		mLogQueue.pop_front();
		mMutex_logQueue.unlock();

		line = "";
		line = line + to_string(entry->timestamp) + "\t";
		line = line + to_string(entry->id) + "\t";
		line = line + entry->str;
		line = line + "\n";
		
		if(mLogStream != NULL)
            *mLogStream << line;

		mMutex_logQueue.lock();
		size = mLogQueue.size();
		mMutex_logQueue.unlock();
	}

	if(mLogStream != NULL)
	{
		mLogStream->close();
		mLogStream = NULL;
	}

	mDone = true;
}

void DataLogger::addLine(const LogIDs &id, const string &str, const double &timestamp)
{

	mMutex_addLine.lock();
	if( mRunning )
	{
		shared_ptr<LogEntry> entry(new LogEntry());
		entry->timestamp = timestamp;
		entry->id = id;
		entry->str = str;

		mMutex_logQueue.lock();
		mLogQueue.push_back(entry);
		mMutex_logQueue.unlock();
	}
	mMutex_addLine.unlock();
}

void DataLogger::addEntry(const LogIDs &id, const string &str, const double &timestamp)
{
	addLine(id, str, timestamp);
}
void DataLogger::addEntry(const LogIDs &id, const Array2D<double> &data, const double &timestamp)
{
    assert( data.dim2() == 1 );
	if( mRunning )
	{
		String str;
		for(int i=0; i<data.dim1(); i++)
			str = str + to_string(data[i][0]) +"\t";

		addLine(id, str, timestamp);
	}
}
void DataLogger::addEntry(const LogIDs &id, const double &data, const double &timestamp)
{
	if( mRunning )
	{
		String str;
        str = str + to_string(data) +"\t";

		addLine(id, str, timestamp);
	}
}

} //hss
