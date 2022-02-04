#include <libcamera/stream.h>
#include <bitset>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#define BITS 32

using Stream = libcamera::Stream;
using ptree = boost::property_tree::ptree;
using bitset = std::bitset<BITS>;

class Digit
{
public:
	Digit() {}
	std::string glyph;
	void Read(ptree const &def);
	void Expand(int factor);
	void SetHints(int offset, int shift);
	int Score(const bitset data[], int rows);
	int Score(const bitset data[], int rows, int offset_hint, int shift_hint);
	int FullScore(const bitset data[], int rows);
	int FullScore(const bitset data[], int rows, int offset_min, int offset_max, int spacing = 0);
	int FullScore(const bitset data[], int rows, int offset_min, int offset_max, int shift_min, int shift_max, int spacing = 0);
	void ShowScore(const bitset data[], int rows) {return ShowScore(data, rows, last_offset, last_shift);}
	void ShowScore(const bitset data[], int rows, int offset, int shift);

	int digit_spacing;
	Digit *prev = NULL;
	Digit *next = NULL;
	bool verbose;

	int last_score;
	int last_offset;
	int last_shift;
	int last_spacing;
private:
	bool print_score_ = false;
	bool has_hints_ = false;
	int n_rows_;
	int width_;
	std::unique_ptr<bitset[]> data_;

	int score_area_(const bitset data[], int rows, int offset, int shift);
	int score_row_(const bitset myrow, const bitset data, int shift);
	void show_score_(const bitset data[], int rows) {return show_score_(data, rows, last_offset, last_shift);}
	void show_score_(const bitset data[], int rows, int offset, int shift);
};

class NumbersStage : public PostProcessingStage
{
public:
	NumbersStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

	bool verbose;
private:
	Stream *stream_;
	Digit digits_[10];
	int n_chars_;
	int h_res_;
	int char_res_;
	int factor_;
	float ftop_, fbottom_;
	float fleft_[2], fright_[2];
	int top_, bottom_;
	int left_[2], right_[2];
	int width_, height_, stride_;
	int charrows_;
	std::unique_ptr<bitset[]> chardata_;
};

#define NAME "numbers"

char const *NumbersStage::Name() const
{
	return NAME;
}

void Digit::Read(ptree const &def)
{
	n_rows_ = def.size();
	data_ = std::make_unique<bitset[]>(n_rows_);
	width_ = 0;
	if (verbose)
		std::cerr << "Reading digit " << glyph << " with " << n_rows_ << " rows:" << std::endl;

	int r = 0;
	for (auto &&row : def)
	{
		bitset rval = 0;
		std::string str = row.second.get_value<std::string>();
		width_ = std::max(width_, (int)str.length());
		for (unsigned int i = 0; i < str.length(); i++) {
			if (str.at(i) == '#')
				rval.set(i);
		}
		data_[r++] = rval;
		if (verbose)
			std::cerr << "- " << (rval << ((BITS - width_) / 2)) << std::endl;
	}
}

void Digit::Expand(int factor) {
	if (verbose)
		std::cerr << "Expanding digit " << glyph << " by " << factor << std::endl;
	auto new_data = std::make_unique<bitset[]>(n_rows_ * factor);
	bitset *ptr = new_data.get();
	for (int r = 0; r < n_rows_; r++) {
		for (int i = 0; i < factor; i++) {
			*ptr++ = data_[r];
			// std::cerr << "+ " << curdata[r] << std::endl;
		}
	}
	data_ = std::move(new_data);
	n_rows_ *= factor;
}


void Digit::SetHints(int offset, int shift) {
	last_offset = offset;
	last_shift = shift;
	has_hints_ = true;
}

int Digit::Score(const bitset data[], int rows) {
	if (has_hints_) {
		return Score(data, rows, has_hints_ ? last_offset : (rows - n_rows_) / 2, has_hints_ ? last_shift : (BITS - width_) / 2);
	} else {
		return FullScore(data, rows);
	}
}

int Digit::Score(const bitset data[], int rows, int offset_hint, int shift_hint) {
	int score = score_area_(data, rows, offset_hint, shift_hint);
	return score;
}
int Digit::FullScore(const bitset data[], int rows) {
	return FullScore(data, rows, -n_rows_ + 1, rows - 1, digit_spacing);
}

int Digit::FullScore(const bitset data[], int rows, int offset_min, int offset_max, int spacing) {
	return FullScore(data, rows, offset_min, offset_max, 0, BITS - width_, spacing);
}

int Digit::FullScore(const bitset data[], int rows, int offset_min, int offset_max, int shift_min, int shift_max, int spacing) {
	int best_score = 0;
	for (int offset = offset_min; offset <= offset_max; offset++) {
		for (int shift = shift_min; shift <= shift_max; shift++) {
			int score = score_area_(data, rows, offset, shift);
			if (score > best_score) {
				best_score = score;
				last_offset = offset;
				last_shift = shift;
			}
		}
	}
	if (spacing) {
		last_spacing = 0;
		Digit *neighbor = NULL;
		if (last_offset < 0) {
			neighbor = next;
		} else if (n_rows_ + last_offset > rows) {
			neighbor = prev;
			spacing = -spacing;
		}
		if (neighbor) {
			best_score += neighbor->FullScore(data, rows, last_offset + spacing * 9/10, last_offset + spacing * 11/10, 0);
			last_spacing = neighbor->last_offset - last_offset;
		}
	}
	last_score = best_score;
	return best_score;
}

int Digit::score_area_(const bitset *data, int rows, int offset, int shift) {
	const bitset *myrow = data_.get();
	int myrows = n_rows_;
	int score = 0;
	if (offset <= 0) {
		// digit is above window
		myrow -= offset;
		myrows += offset;
	} else {
		data += offset;
		rows -= offset;
	}
	int count = std::min(myrows, rows);
	// std::cerr << "score " << rows << " rows @"<<shift<<","<<offset<<std::endl;
	while (count-- > 0) {
		score += score_row_(*myrow++, *data++, shift);
	}
	return score;
}

int Digit::score_row_(const bitset myrow, const bitset data, int shift) {
	bitset shiftrow = myrow << shift;
	bitset xorrow = shiftrow ^ data;
	int expectednotfound = (shiftrow & xorrow).count();
	int expectedandfound = (shiftrow & ~xorrow).count();
	// int score = BITS - xorrow.count() - expectednotfound * 4 + expectedandfound;
	int score = expectedandfound - expectednotfound * 2;
	if (print_score_) {
		std::cout << "  ";
		for (int i = 0; i < BITS; i++) {
			if (!xorrow.test(i)) {
				// match in this position
				std::cout << (shiftrow.test(i) ? "#######" : ".......");
			} else if (shiftrow.test(i)) {
				// expected pixel in this position, not found
				std::cout << "xxxxxxx";
			} else {
				// expected blank in this position, found pixel
				std::cout << "_______";
			}
		}
		// std::cout << "  " << shiftrow << " ^ " << data << " = " << xorrow << " (" << score << ")" << std::endl;
		// printf(" (%d = %d - %d - %d * 4 + %d)\n", score, BITS, (int)xorrow.count(), expectednotfound, expectedandfound);
		printf(" (%d = %d - %d * 2)\n", score, expectedandfound, expectednotfound);
		// std::cout << " (" << score << ")" << std::endl;
	}
	return score;
}

void Digit::ShowScore(const bitset data[], int rows, int offset, int shift) {
	FullScore(data, rows, offset, offset, shift, shift, digit_spacing);
	if (last_spacing < 0) {
		prev->show_score_(data, rows);
		std::cout << std::endl;
	}
	show_score_(data, rows, offset, shift);
	if (last_spacing > 0) {
		std::cout << std::endl;
		next->show_score_(data, rows);
	}
}

void Digit::show_score_(const bitset data[], int rows, int offset, int shift) {
	print_score_ = true;
	score_area_(data, rows, offset, shift);
	print_score_ = false;
}

void NumbersStage::Read(ptree const &params)
{
	verbose = params.get<bool>("verbose", false);
	n_chars_ = params.get<int>("n_chars", 1);
	h_res_ = params.get<int>("h_res", 16);
	if (h_res_ < 1 || h_res_ > 32)
		throw std::runtime_error("NumbersStage: h_res must be between 1 and 32");
	ftop_ = params.get<float>("top", 0.0);
	fbottom_ = params.get<float>("bottom", 1.0);

	float left = params.get<float>("left", 0.0);
	float right = params.get<float>("right", 1.0);

	fleft_[0] = params.get<float>("left.top", left);
	fleft_[1] = params.get<float>("left.bottom", left);
	fright_[0] = params.get<float>("right.top", right);
	fright_[1] = params.get<float>("right.bottom", right);
	if (verbose)
		printf("n_chars = %d\nh_res = %d\ntop = %f\nbottom = %f\nleft = %f,%f\nright = %f,%f\n", n_chars_, h_res_, ftop_, fbottom_, fleft_[0], fleft_[1], fright_[0], fright_[1]);

	for (int i = 0; i < 10; i++) {
		digits_[i].verbose = verbose;
		digits_[i].next = digits_ + (i + 1) % 10;
		digits_[i].prev = digits_ + (i + 9) % 10;
	}

	char_res_ = params.get<int>("char_res", h_res_);
	for (auto &&cdata : params.get_child("chars")) {
		auto cname = cdata.first;
		auto cdef = cdata.second;
		int n = -1;
		if (cname.length() == 1) {
			n = cname.at(0) - '0';
		}
		if (n < 0 || n > 9) {
			throw std::runtime_error("NumbersStage: chars only supports 0 through 9");
		}
		digits_[n].glyph = cname;
		digits_[n].Read(cdef);
	}
}

void NumbersStage::Configure()
{
	stream_ = app_->GetMainStream();
	if (!stream_ || stream_->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("NumbersStage: only YUV420 format supported");
	auto config = stream_->configuration();
	auto pixfmt = config.pixelFormat;
	width_ = config.size.width;
	height_ = config.size.height;
	stride_ = config.stride;
	left_[0] = fleft_[0] * width_;
	left_[1] = fleft_[1] * width_;
	right_[0] = fright_[0] * width_;
	right_[1] = fright_[1] * width_;
	top_ = ftop_ * height_;
	bottom_ = fbottom_ * height_;
	charrows_ = bottom_ - top_;
	chardata_ = std::make_unique<bitset[]>(n_chars_ * charrows_);
	factor_ = (right_[0] - left_[0]) / n_chars_ / h_res_;
	if (verbose)
		printf("size: %s, stride: %d, frameSize: %d, pixfmt: %s, factor: %d\n", config.size.toString().c_str(), config.stride, config.frameSize, pixfmt.toString().c_str(), factor_);
	for (int n = 0; n < 10; n++) {
		digits_[n].Expand(factor_);
		digits_[n].digit_spacing = charrows_;
	}
}

bool NumbersStage::Process(CompletedRequestPtr &completed_request)
{
	libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request->buffers[stream_])[0];
	int rows = charrows_;
	uint8_t *row = buffer.data() + (top_ * stride_);

	for (int i = 0; i < n_chars_ * rows; i++) {
		chardata_[i] = 0;
	}

	if (verbose)
		printf("n_chars = %d\ntop = %d\nbottom=%d\nleft=%d,%d\nright=%d,%d\n", n_chars_, top_, bottom_, left_[0], left_[1], right_[0], right_[1]);

	// y = top;
	// for (x = left_[0]; x < right_[0]; x++)
	// 	row[x] = 255;

	for (int r = 0; r < rows; r++) {
		row += stride_;

		int left = left_[0] + (left_[1] - left_[0]) * r / rows;
		int right = right_[0] + (right_[1] - right_[0]) * r / rows;
		int cwidth = (right - left) / n_chars_;
		int pwidth = cwidth / h_res_;

		for (int i = 0; i < n_chars_; i++) {
			int cleft = left + (right - left) * i / n_chars_ + pwidth / 2;
			for (int p = 0; p < h_res_; p++) {
				bool bit = row[cleft + p * pwidth] == 0xFF;
				chardata_[i * rows + r].set(p, bit);
				row[p + i*h_res_] = bit ? 0xFF : 0;
			}
			row[cleft - pwidth / 2] = 0xFF;
		}

		row[right] = 0xFF;
	}

	// ptr += stride_;
	// for (x = left_[1]; x < right_[1]; x++)
	// 	*ptr++ = 255;

	for (int i = 0; i < n_chars_; i++) {
		int best_score = 0;
		Digit *best_digit = NULL;
		bitset *chardata = &chardata_[i * rows];
		if (verbose) {
			std::cout << "Char " << i << ":" << std::endl;
			for (int r = 0; r < rows; r += factor_) {
				for (int p = 0; p < h_res_; p++) {
					int bit = chardata[r].test(p);
					std::cout << (bit ? '#' : '.');
				}
				std::cout << std::endl;
			}
			std::cout << "Scoring:" << std::endl;
		}
		for (Digit &dig : digits_) {
			int score = dig.FullScore(chardata, rows);
			if (score > best_score) {
				best_score = score;
				best_digit = &dig;
			}
			if (verbose) {
				std::cout << " digit " << dig.glyph << " score: " << score;
				if (dig.last_spacing) {
					int neighbor_score = (dig.last_spacing < 0 ? dig.prev : dig.next)->last_score;
					std::cout << " (" << (score - neighbor_score) << " + " << neighbor_score << ")";
				}
				std::cout << " @" << dig.last_shift << "," << dig.last_offset;
				if (dig.last_spacing) {
					printf(" (%+d)", dig.last_spacing);
				}
				std::cout << std::endl;
			}
		}
		auto &&bd = *best_digit;
		if (verbose) {
			std::cout << "best guess: " << bd.glyph << " @" << bd.last_shift << "," << bd.last_offset << "/" << (bd.last_offset / factor_) << std::endl;
			bd.ShowScore(chardata, rows);
			std::cout << std::endl;
		} else {
			std::cout << bd.glyph;
		}
	}

	if (verbose)
		printf("done\n");
	else
		std::cout << std::endl;

	return false;
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new NumbersStage(app);
}

static RegisterStage reg(NAME, &Create);
