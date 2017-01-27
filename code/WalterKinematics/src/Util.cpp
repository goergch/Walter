
#include <stdlib.h>
#include <chrono>
#include <unistd.h>
#include <thread>
#include <iomanip>

#include "setup.h"
#include "Util.h"
#include <cstdarg>
#include "logger.h"

float roundValue(float x) {
	float roundedValue = sgn(x)*((int)(abs(x)*10.0f+.5f))/10.0f;
	return roundedValue;
}

string getPath(string uri) {
	int idx = upcase(uri).find("?");
	if (idx >= 0)
		return uri.substr(0,idx);

	return "";
}

bool hasPrefix(string str, string prefix) {
	int idx = upcase(str).find(upcase(prefix));
	return (idx>=0);
}

struct HTMLReplace {
  string match;
  string replace;
} codes[] = {
  {"&", "&amp;"},
  {"\"","&quot;"},
  {"'", "&apos;"},
  {"<", "&lt;"},
  {">", "&gt;"},
  {"\t", "&#9;"}
};


#define array_length(array) (sizeof (array) / sizeof (array)[0])
string htmlEncode( const string s )
{
  string rs = s;
  // Replace each matching token in turn
  for ( size_t i = 0; i < array_length( codes ); i++ ) {
    // Find the first match
    const string& match = codes[i].match;
    const string& repl = codes[i].replace;
    string::size_type start = rs.find_first_of( match );
    // Replace all matches
    while ( start != string::npos ) {
      rs.replace( start, match.size(), repl );
      // Be sure to jump forward by the replacement length
      start = rs.find_first_of( match, start + repl.size() );
    }
  }
  return rs;
}

string htmlDecode( string s )
{
  string rs = s;
  // Replace each matching token in turn
  for ( size_t i = 0; i < array_length( codes ); i++ ) {
    // Find the first match
    const string& match = codes[i].replace;
    const string& repl = codes[i].match;
    string::size_type start = rs.find_first_of( match );
    // Replace all matches
    while ( start != string::npos ) {
      rs.replace( start, match.size(), repl );
      // Be sure to jump forward by the replacement length
      start = rs.find_first_of( match, start + repl.size() );
    }
  }
  return rs;
}

void urldecode_c(char *dst, const char *src)
{
        char a, b;
        while (*src) {
                if ((*src == '%') &&
                    ((a = src[1]) && (b = src[2])) &&
                    (isxdigit(a) && isxdigit(b))) {
                        if (a >= 'a')
                                a -= 'a'-'A';
                        if (a >= 'A')
                                a -= ('A' - 10);
                        else
                                a -= '0';
                        if (b >= 'a')
                                b -= 'a'-'A';
                        if (b >= 'A')
                                b -= ('A' - 10);
                        else
                                b -= '0';
                        *dst++ = 16*a+b;
                        src+=3;
                } else if (*src == '+') {
                        *dst++ = ' ';
                        src++;
                } else {
                        *dst++ = *src++;
                }
        }
        *dst++ = '\0';
}

string urlEncode(const string &value) {
    ostringstream escaped;
    escaped.fill('0');
    escaped << hex;

    for (string::const_iterator i = value.begin(), n = value.end(); i != n; ++i) {
        string::value_type c = (*i);

        // Keep alphanumeric and other accepted characters intact
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            escaped << c;
            continue;
        }

        // Any other characters are percent-encoded
        escaped << uppercase;
        escaped << '%' << setw(2) << int((unsigned char) c);
        escaped << nouppercase;
    }

    return escaped.str();
}
string urlDecode(string input) {
	char target[input.length()*3];
	urldecode_c(&target[0], input.c_str());
	return string(target);
}

std::string string_format(const std::string &fmt, ...) {
    int size=100;
    std::string str;
    va_list ap;

    while (1) {
        str.resize(size);
        va_start(ap, fmt);
        int n = vsnprintf(&str[0], size, fmt.c_str(), ap);
        va_end(ap);

        if (n > -1 && n < size) {
            string result;
            result.append(str,0,n);
            return result;
        }
        if (n > -1)
            size = n + 1;
        else
            size *= 2;
    }
}

long mapLong(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int randomInt(int min,int max) {
	int r = rand() % (max-min) + min;
	return r;
}

rational randomFloat (rational a, rational b) {
	return randomInt(a*1000, b*1000)/1000.;
}

bool randomBool() {
	return randomInt(0,100)>50;
}

int randomPosNeg() {
	return (randomInt(0,100)>50)?+1:-1;
}

milliseconds millis() {
    auto epoch = std::chrono::high_resolution_clock::from_time_t(0);
    auto now   = std::chrono::high_resolution_clock::now();
    auto mseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - epoch).count();
    return mseconds;
}

string currentTimeToString()
{
    time_t nowTime = time(0);
    tm *ltm = localtime(&nowTime);
    auto epoch = std::chrono::high_resolution_clock::from_time_t(0);
    auto now   = std::chrono::high_resolution_clock::now();

    string r;stringstream s;
    s << string("") + ((ltm->tm_hour<10)?"0":"") <<  ltm->tm_hour << ":"
      << string("") + ((ltm->tm_min<10)?"0":"") << ltm->tm_min << ":"
      << string("") + ((ltm->tm_sec<10)?"0":"") << ltm->tm_sec << ","
      << std::chrono::duration_cast<std::chrono::milliseconds>(now - epoch).count() % 1000;

    r = s.str();
    return r;
}

void delay(long ms) {
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void delay_us(long us) {
	std::this_thread::sleep_for(std::chrono::nanoseconds(us*1000L));
}

bool string_starts_with(string s, string start) {
	return (strncmp(s.c_str(), start.c_str(), strlen(start.c_str())) == 0);
}

string to_string(rational number, int precision) {
	return
			static_cast< std::ostringstream & >(
					(std::ostringstream() << std::setprecision(precision) <<  number)
			).str();
}

string to_string(int number) {
	return
			static_cast< std::ostringstream & >(
					(std::ostringstream() <<  number)
			).str();
}


int string_to_int (const string &str) {
  stringstream ss(str);
  int num;
  if((ss >> num).fail())
  {
      return -1;
  }
  return num;
}

string upcase(string str) {
	string result(str);
	std::transform(result.begin(), result.end(),result.begin(), ::toupper);
	return result;
}

string dncase(string str) {
	string result(str);
	std::transform(result.begin(), result.end(),result.begin(), ::tolower);
	return result;
}

string int_to_string(int x) {
       string r;stringstream s;
       s << x;
       r = s.str();
       return r;
}

void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            std::not1(std::ptr_fun<int, int>(std::isspace))));
}// trim from start (in place)

void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
            std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
} // trim from end (in place)

void trim(std::string &s) {
	ltrim(s);
    rtrim(s);
}

std::string replaceWhiteSpace(std::string s) {
	replace (s.begin(), s.end(), '\r' , 'R');
	replace (s.begin(), s.end(), '\n' , 'N');
	return s;
}


rational hypothenuseLength(rational a, rational b) {
    return sqrt(a*a+b*b);
}

rational radians(rational degrees) {
	const rational fac = (PI/ 180.0);
	return degrees * fac;
}

rational  degrees(rational radians) {
	const rational fac = (180.0/ PI);
	return radians * fac;
}

// cosine sentence
rational triangleAlpha(rational a, rational b, rational c) {
	rational x = acos((a*a-b*b-c*c)/(-2.0*b*c));
    return x;
}

// cosine sentence
rational triangleGamma(rational a, rational b, rational c) {
	return triangleAlpha(c,b,a);
}

// abc formular, root of 0 = a*x*x + b*x + c;
bool polynomRoot2ndOrder(rational a, rational b, rational c, rational& root0, rational& root1)
{
	rational disc = b*b-4.0*a*c;
	if (disc>=0) {
		root0 = (-b + sqrt(disc)) / (2.0*a);
		root1 = (-b - sqrt(disc)) / (2.0*a);
		return true;
	}
	return false;
}

bool almostEqual(rational a, rational b, rational precision) {
	if (a==b)
		return true;
	if (a == 0)
		return (abs(b)<precision);
	if (b == 0)
		return (abs(a)<precision);

	if (b<a)
		return (abs((b/a)-1.0) < precision);
	else
		return (abs((a/b)-1.0) < precision);

}

bool fileExists(const string& fileName) {
    ifstream file;

    file.open(fileName.c_str());
    if(file.is_open()) {
    	file.close();
    	return true;
    }
    return false;
}

#ifdef _WIN32

vector<std::string> readDirectory(const string & dir, const string& ext) {
  string item;
  int i = 0;

  vector<string> result;
  string filepattern = "*.";
  filepattern += ext;

#ifdef _WIN32

  WIN32_FIND_DATA FN;
  HANDLE hFind;

  string path = "*." + ext;
  hFind = FindFirstFile(path.c_str(), &FN);
    if (hFind != INVALID_HANDLE_VALUE) {
      do {
        string item;

        if (FN.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
        	item = '\\';
        	item += FN.cFileName;
        } else {
        	item = FN.cFileName;
        }
        	result.push_back(item);
        i++;
      } while (FindNextFile(hFind, &FN) != 0);

      if (GetLastError() == ERROR_NO_MORE_FILES)
        FindClose(hFind);
      else
        LOG(ERROR) << "failed read dir";
    }

#elif defined(__GNUC__)

  DIR *dir;
  struct dirent *dirp;
  struct stat dr;

  if (list) {
    list->delete_all();
    if ((dir = opendir(d)) == NULL)
      LOG(ERROR) << "failed read dir";
    else {
      while ((dirp = readdir(dir)) != NULL)   /* open directory     */
      {
    	 string item;
        if (!lstat(dirp->d_name,&dr) && S_ISDIR(dr.st_mode)) /* dir is directory   */
          item = dirp->d_name + "/";
        else
          item = dirp->d_name;

        result.push_back(item);
        i++;
      }
      closedir(dir);
    }
  }
#endif

  return result;
}
#endif

void eatWhiteSpace(string s, int & idx) {
	while ((idx < (int)s.size()) && ((s[idx] == ' ') || (s[idx] == '\n') || (s[idx] == '\r') || (s[idx] == '\t')))
		idx++;
}


string endofline(int indent) {
	string s = "\n";
	for (int i = 0;i<indent;i++)
		s += "  ";
	return s;
}

string floatToString(const string& tag, double x) {
	stringstream str;
	str.precision(10);
	str << tag << "=" << std::fixed << x << " ";
	return str.str();
}

bool floatFromString (const string& tag, const string& str, double &x, int& idx) {
	string parseStr = tag + "=%lf %n";
	int tmpIdx;
	eatWhiteSpace(str,idx);
    int noOfItems = sscanf(str.substr(idx).c_str(),parseStr.c_str(), &x, &tmpIdx);
    bool ok = (noOfItems == 1);
    if (ok)
        idx += tmpIdx;
    return ok;
}

string intToString(const string& tag, int x) {
	stringstream str;
	str.precision(3);
	str << tag << "=" << x << " ";
	return str.str();
}

bool intFromString (const string& tag, const string& str, int &x, int& idx) {
	string parseStr = tag + "=%i%n";
	int tmpIdx;
	string s = str.substr(idx);
	eatWhiteSpace(str,idx);
    int noOfItems = sscanf(str.substr(idx).c_str(),parseStr.c_str(), &x, &tmpIdx);
    bool ok = (noOfItems == 1);
    if (ok)
        idx += tmpIdx;

    return ok;
}

string boolToString(const string& tag, bool x) {
	stringstream str;
	str.precision(3);
	str << tag << "=" << (int)(x?1:0) << " ";
	return str.str();
}

bool boolFromString (const string& tag, const string& str, bool &x, int& idx) {
	string parseStr = tag + "=%i%n";
	int tmpIdx;
	eatWhiteSpace(str,idx);
	string s = str.substr(idx);
	int dummyInt;
    int noOfItems = sscanf(str.substr(idx).c_str(),parseStr.c_str(), &dummyInt, &tmpIdx);
    x = (dummyInt == 1);
    bool ok = (noOfItems == 1);
    if (ok)
        idx += tmpIdx;

    return ok;
}


string uint32ToString(const string& tag, uint32_t x) {
	stringstream str;
	str.precision(3);
	str << tag << "=" << x << " ";
	return str.str();
}

bool uint32FromString (const string& tag, const string& str, uint32_t &x, int& idx) {
	string parseStr = tag + "=%u%n";
	int tmpIdx;
	eatWhiteSpace(str,idx);
	string s = str.substr(idx);
    int noOfItems = sscanf(str.substr(idx).c_str(),parseStr.c_str(), &x, &tmpIdx);
    bool ok = (noOfItems == 1);
    if (ok)
        idx += tmpIdx;

    return ok;
}

string stringToString(const string& tag, const string& x) {
	stringstream str;
	str.precision(3);
	string encoded = string_to_hex(string("-")+x);

	str << tag << "=" << encoded << " ";
	return str.str();
}

bool stringFromString (const string& tag, const string& str, string &x, int& idx) {
	string parseStr = tag + "=%s%n";
	int tmpIdx;
	eatWhiteSpace(str,idx);
	string s = str.substr(idx);
	char buffer[256];
    int noOfItems = sscanf(str.substr(idx).c_str(),parseStr.c_str(), buffer, &tmpIdx);
    x = buffer;
    x = hex_to_string(x);
    x = x.substr(1);
    idx += tmpIdx;
    bool ok = (noOfItems == 1);
    return ok;
}

string listStartToString(const string& tag, int &indent) {
	stringstream str;
	str << tag << " {";
	indent++;
	return str.str();
}

string listEndToString(int& indent) {
	stringstream str;
	indent--;
	str << "}";
	return str.str();
}
bool listStartFromString (const string& tag, const string& str, int& idx) {
	string parseStr = tag + " {%n";
	int tmpIdx;
	eatWhiteSpace(str,idx);
	string s = str.substr(idx);
    bool ok = false;
	if (string_starts_with(s,tag)) {
		int noOfItems = sscanf(str.substr(idx).c_str(),parseStr.c_str(),  &tmpIdx);
		ok = (noOfItems == 0);
		if (ok)
			idx += tmpIdx;
	}
    return ok;
}

bool listEndFromString (const string& str, int& idx) {
	int tmpIdx;
	eatWhiteSpace(str,idx);
	string s = str.substr(idx);
	bool ok = false;
	if (string_starts_with(s,"}")) {
		int noOfItems = sscanf(str.substr(idx).c_str(),"}%n", &tmpIdx);
		ok = (noOfItems == 0);
		if (ok)
			idx += tmpIdx;
	}
    return ok;
}

std::string string_to_hex(const std::string& input)
{
    static const char* const lut = "0123456789ABCDEF";
    size_t len = input.length();

    std::string output;
    output.reserve(2 * len);
    for (size_t i = 0; i < len; ++i)
    {
        const unsigned char c = input[i];
        output.push_back(lut[c >> 4]);
        output.push_back(lut[c & 15]);
    }
    return output;
}



std::string hex_to_string(const std::string& input)
{
    static const char* const lut = "0123456789ABCDEF";
    size_t len = input.length();
    if (len & 1) throw std::invalid_argument("odd length");

    std::string output;
    output.reserve(len / 2);
    for (size_t i = 0; i < len; i += 2)
    {
        char a = input[i];
        const char* p = std::lower_bound(lut, lut + 16, a);
        if (*p != a) throw std::invalid_argument("not a hex digit");

        char b = input[i + 1];
        const char* q = std::lower_bound(lut, lut + 16, b);
        if (*q != b) throw std::invalid_argument("not a hex digit");

        output.push_back(((p - lut) << 4) | (q - lut));
    }
    return output;
}
