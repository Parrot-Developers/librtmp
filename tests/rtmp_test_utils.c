/**
 * Copyright (c) 2018 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "rtmp_test.h"


static void test_rtmp_anonymize_url(void)
{
	int ret = 0;
	char *anonymized = NULL;
	struct url_anonymized_map {
		const char *url;
		const char *anon;
	};

	struct url_anonymized_map test_cases_ko_map[] = {
		{"a.rtmp.youtube.com/live2/AaBb-CcDd-EeFf-GgHh-IiJj", NULL},
		{"http://a.rtmp.youtube.com/live2/AaBb-CcDd-EeFf-GgHh-IiJj",
		 NULL},
		{"ftp://a.rtmp.youtube.com/live2/AaBb-CcDd-EeFf-GgHh-IiJj",
		 NULL},
		{"rtmp://a.rtmp.youtube.com/live2/", NULL},
	};

	struct url_anonymized_map test_cases_ok_map[] = {
		/* YouTube Live */
		{"rtmp://a.rtmp.youtube.com/live2/AaBb-CcDd-EeFf-GgHh-IiJj",
		 "rtmp://a.rtmp.youtube.com/li*e2/Aa********************Jj"},
		/* YouTube Live (+ port) */
		{"rtmp://a.rtmp.youtube.com:1935/live2/"
		 "AaBb-CcDd-EeFf-GgHh-IiJj",
		 "rtmp://a.rtmp.youtube.com:1935/li*e2/"
		 "Aa********************Jj"},
		/* YouTube Live (RTMPS) */
		{"rtmps://a.rtmps.youtube.com:443/live2/"
		 "AaBb-CcDd-EeFf-GgHh-IiJj",
		 "rtmps://a.rtmps.youtube.com:443/li*e2/"
		 "Aa********************Jj"},
		/* Wowza */
		{"rtmp://AaBbCcDdEeFf.entrypoint.cloud.wowza.com/"
		 "app-AaBbCcDd/EeFfGgHh",
		 "rtmp://AaBbCcDdEeFf.entrypoint.cloud.wowza.com/"
		 "ap********Dd/Ee****Hh"},
		/* Wowza (+ port)*/
		{"rtmp://AaBbCcDdEeFf.entrypoint.cloud.wowza.com:1935/"
		 "app-AaBbCcDd/EeFfGgHh",
		 "rtmp://AaBbCcDdEeFf.entrypoint.cloud.wowza.com:1935/"
		 "ap********Dd/Ee****Hh"},
		/* Wowza */
		{"rtmp://AaBbCcDdEeFf.wowza.com/app-AaBbCcDd/EeFfGgHh",
		 "rtmp://AaBbCcDdEeFf.wowza.com/ap********Dd/Ee****Hh"},
	};

	ret = rtmp_anonymize_uri(NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	CU_ASSERT_PTR_NULL(anonymized);

	/* KO */
	for (size_t i = 0; i < ARRAY_SIZE(test_cases_ko_map); i++) {
		anonymized = NULL;
		ret = rtmp_anonymize_uri(test_cases_ko_map[i].url, &anonymized);
		CU_ASSERT_EQUAL(ret, -EPROTO);
		CU_ASSERT_PTR_NULL(anonymized);
	}

	/* OK */
	for (size_t i = 0; i < ARRAY_SIZE(test_cases_ok_map); i++) {
		anonymized = NULL;
		ret = rtmp_anonymize_uri(test_cases_ok_map[i].url, &anonymized);
		CU_ASSERT_EQUAL(ret, 0);
		CU_ASSERT_PTR_NOT_NULL(anonymized);
		CU_ASSERT_STRING_EQUAL(anonymized, test_cases_ok_map[i].anon);
		printf("'%s'\n'%s'\n", anonymized, test_cases_ok_map[i].anon);
		free(anonymized);
	}
}


CU_TestInfo g_rtmp_test_utils[] = {
	{FN("rtmp-anonymize-url"), &test_rtmp_anonymize_url},

	CU_TEST_INFO_NULL,
};
